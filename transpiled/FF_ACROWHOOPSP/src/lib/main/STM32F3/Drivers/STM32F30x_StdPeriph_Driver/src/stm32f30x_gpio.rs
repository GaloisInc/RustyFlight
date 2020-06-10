use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_AHBPeriphResetCmd(RCC_AHBPeriph: uint32_t,
                             NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint16_t,
    pub RESERVED0: uint16_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint16_t,
    pub RESERVED1: uint16_t,
    pub ODR: uint16_t,
    pub RESERVED2: uint16_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
    pub BRR: uint16_t,
    pub RESERVED3: uint16_t,
}
pub type GPIOMode_TypeDef = libc::c_uint;
pub const GPIO_Mode_AN: GPIOMode_TypeDef = 3;
pub const GPIO_Mode_AF: GPIOMode_TypeDef = 2;
pub const GPIO_Mode_OUT: GPIOMode_TypeDef = 1;
pub const GPIO_Mode_IN: GPIOMode_TypeDef = 0;
pub type GPIOOType_TypeDef = libc::c_uint;
pub const GPIO_OType_OD: GPIOOType_TypeDef = 1;
pub const GPIO_OType_PP: GPIOOType_TypeDef = 0;
pub type GPIOSpeed_TypeDef = libc::c_uint;
pub const GPIO_Speed_Level_3: GPIOSpeed_TypeDef = 3;
pub const GPIO_Speed_Level_2: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_Level_1: GPIOSpeed_TypeDef = 1;
pub type GPIOPuPd_TypeDef = libc::c_uint;
pub const GPIO_PuPd_DOWN: GPIOPuPd_TypeDef = 2;
pub const GPIO_PuPd_UP: GPIOPuPd_TypeDef = 1;
pub const GPIO_PuPd_NOPULL: GPIOPuPd_TypeDef = 0;
pub type BitAction = libc::c_uint;
pub const Bit_SET: BitAction = 1;
pub const Bit_RESET: BitAction = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint32_t,
    pub GPIO_Mode: GPIOMode_TypeDef,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_OType: GPIOOType_TypeDef,
    pub GPIO_PuPd: GPIOPuPd_TypeDef,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the GPIO peripheral:
  *           + Initialization and Configuration functions
  *           + GPIO Read and Write functions
  *           + GPIO Alternate functions configuration functions
  *
  *  @verbatim


 ===============================================================================
                      ##### How to use this driver #####
 ===============================================================================
    [..]
    (#) Enable the GPIO AHB clock using RCC_AHBPeriphClockCmd()
    (#) Configure the GPIO pin(s) using GPIO_Init()
        Four possible configuration are available for each pin:
        (++) Input: Floating, Pull-up, Pull-down.
        (++) Output: Push-Pull (Pull-up, Pull-down or no Pull),
                     Open Drain (Pull-up, Pull-down or no Pull).
             In output mode, the speed is configurable: Low, Medium, Fast or High.
        (++) Alternate Function: Push-Pull (Pull-up, Pull-down or no Pull), 
                                 Open Drain (Pull-up, Pull-down or no Pull).
        (++) Analog: required mode when a pin is to be used as ADC channel,
             DAC output or comparator input.
    (#) Peripherals alternate function:
        (++) For ADC, DAC and comparators, configure the desired pin in 
             analog mode using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AN
        (++) For other peripherals (TIM, USART...):
             (+++) Connect the pin to the desired peripherals' Alternate 
                   Function (AF) using GPIO_PinAFConfig() function.
             (+++) Configure the desired pin in alternate function mode using
                   GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
             (+++) Select the type, pull-up/pull-down and output speed via 
                   GPIO_PuPd, GPIO_OType and GPIO_Speed members.
             (+++) Call GPIO_Init() function.
    (#) To get the level of a pin configured in input mode use GPIO_ReadInputDataBit()
    (#) To set/reset the level of a pin configured in output mode use
        GPIO_SetBits()/GPIO_ResetBits()
    (#) During and just after reset, the alternate functions are not active 
        and the GPIO pins are configured in input floating mode (except JTAG pins).
    (#) The LSE oscillator pins OSC32_IN and OSC32_OUT can be used as 
        general-purpose (PC14 and PC15, respectively) when the LSE
        oscillator is off. The LSE has priority over the GPIO function.
    (#) The HSE oscillator pins OSC_IN/OSC_OUT can be used as general-purpose 
        (PF0 and PF1 respectively) when the HSE oscillator is off. The HSE has 
        the priority over the GPIO function.  

  @endverbatim

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
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F30x_StdPeriph_Driver
  * @{
  */
/* * @defgroup GPIO 
  * @brief GPIO driver modules
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup GPIO_Private_Functions 
  * @{
  */
/* * @defgroup GPIO_Group1 Initialization and Configuration
 *  @brief   Initialization and Configuration
 *
@verbatim
 ===============================================================================
            ##### Initialization and Configuration #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the GPIOx peripheral registers to their default reset 
  *         values.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_DeInit(mut GPIOx: *mut GPIO_TypeDef) {
    /* Check the parameters */
    if GPIOx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut GPIO_TypeDef {
        RCC_AHBPeriphResetCmd(0x20000 as libc::c_int as uint32_t, ENABLE);
        RCC_AHBPeriphResetCmd(0x20000 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_AHBPeriphResetCmd(0x40000 as libc::c_int as uint32_t, ENABLE);
        RCC_AHBPeriphResetCmd(0x40000 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x800
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_AHBPeriphResetCmd(0x80000 as libc::c_int as uint32_t, ENABLE);
        RCC_AHBPeriphResetCmd(0x80000 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0xc00
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_AHBPeriphResetCmd(0x100000 as libc::c_int as uint32_t, ENABLE);
        RCC_AHBPeriphResetCmd(0x100000 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1000
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_AHBPeriphResetCmd(0x200000 as libc::c_int as uint32_t, ENABLE);
        RCC_AHBPeriphResetCmd(0x200000 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_AHBPeriphResetCmd(0x400000 as libc::c_int as uint32_t, ENABLE);
        RCC_AHBPeriphResetCmd(0x400000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the GPIOx peripheral according to the specified 
  *         parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that 
  *         contains the configuration information for the specified GPIO
  *         peripheral.
  * @note   GPIO_Pin: selects the pin to be configured:
  *         GPIO_Pin_0->GPIO_Pin_15 for GPIOA, GPIOB, GPIOC, GPIOD and GPIOE;
  *         GPIO_Pin_0->GPIO_Pin_2, GPIO_Pin_4, GPIO_Pin_6, GPIO_Pin_9 
  *                       and GPIO_Pin_10 for GPIOF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_Init(mut GPIOx: *mut GPIO_TypeDef,
                                   mut GPIO_InitStruct:
                                       *mut GPIO_InitTypeDef) {
    let mut pinpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut currentpin: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*-------------------------- Configure the port pins -----------------------*/
  /*-- GPIO Mode Configuration --*/
    pinpos = 0 as libc::c_int as uint32_t;
    while pinpos < 0x10 as libc::c_int as libc::c_uint {
        pos = (0x1 as libc::c_int as uint32_t) << pinpos;
        /* Get the port pins position */
        currentpin = (*GPIO_InitStruct).GPIO_Pin & pos;
        if currentpin == pos {
            if (*GPIO_InitStruct).GPIO_Mode as libc::c_uint ==
                   GPIO_Mode_OUT as libc::c_int as libc::c_uint ||
                   (*GPIO_InitStruct).GPIO_Mode as libc::c_uint ==
                       GPIO_Mode_AF as libc::c_int as libc::c_uint {
                /* Check Speed mode parameters */
                /* Speed mode configuration */
                ::core::ptr::write_volatile(&mut (*GPIOx).OSPEEDR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).OSPEEDR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !((0x3 as libc::c_int as
                                                        uint32_t) <<
                                                       pinpos.wrapping_mul(2
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint)))
                                                as uint32_t as uint32_t);
                ::core::ptr::write_volatile(&mut (*GPIOx).OSPEEDR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).OSPEEDR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 ((*GPIO_InitStruct).GPIO_Speed
                                                      as uint32_t) <<
                                                     pinpos.wrapping_mul(2 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint))
                                                as uint32_t as uint32_t);
                /* Check Output mode parameters */
                /* Output mode configuration */
                ::core::ptr::write_volatile(&mut (*GPIOx).OTYPER as
                                                *mut uint16_t,
                                            (::core::ptr::read_volatile::<uint16_t>(&(*GPIOx).OTYPER
                                                                                        as
                                                                                        *const uint16_t)
                                                 as libc::c_uint &
                                                 !((0x1 as libc::c_int as
                                                        uint32_t) <<
                                                       pinpos as uint16_t as
                                                           libc::c_int)) as
                                                uint16_t as uint16_t);
                ::core::ptr::write_volatile(&mut (*GPIOx).OTYPER as
                                                *mut uint16_t,
                                            (::core::ptr::read_volatile::<uint16_t>(&(*GPIOx).OTYPER
                                                                                        as
                                                                                        *const uint16_t)
                                                 as libc::c_int |
                                                 (((*GPIO_InitStruct).GPIO_OType
                                                       as uint16_t as
                                                       libc::c_int) <<
                                                      pinpos as uint16_t as
                                                          libc::c_int) as
                                                     uint16_t as libc::c_int)
                                                as uint16_t as uint16_t)
            }
            ::core::ptr::write_volatile(&mut (*GPIOx).MODER as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).MODER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !((0x3 as libc::c_int as
                                                    uint32_t) <<
                                                   pinpos.wrapping_mul(2 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint)))
                                            as uint32_t as uint32_t);
            ::core::ptr::write_volatile(&mut (*GPIOx).MODER as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).MODER
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             ((*GPIO_InitStruct).GPIO_Mode as
                                                  uint32_t) <<
                                                 pinpos.wrapping_mul(2 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint))
                                            as uint32_t as uint32_t);
            /* Use temporary variable to update PUPDR register configuration, to avoid 
         unexpected transition in the GPIO pin configuration. */
            tmpreg = (*GPIOx).PUPDR;
            tmpreg &=
                !((0x3 as libc::c_int as uint32_t) <<
                      pinpos as uint16_t as libc::c_int * 2 as libc::c_int);
            tmpreg |=
                ((*GPIO_InitStruct).GPIO_PuPd as uint32_t) <<
                    pinpos.wrapping_mul(2 as libc::c_int as libc::c_uint);
            ::core::ptr::write_volatile(&mut (*GPIOx).PUPDR as *mut uint32_t,
                                        tmpreg)
        }
        pinpos = pinpos.wrapping_add(1)
    };
}
/* *
  * @brief  Fills each GPIO_InitStruct member with its default value.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure which will 
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_StructInit(mut GPIO_InitStruct:
                                             *mut GPIO_InitTypeDef) {
    /* Reset GPIO init structure parameters values */
    (*GPIO_InitStruct).GPIO_Pin =
        0xffff as libc::c_int as uint16_t as uint32_t;
    (*GPIO_InitStruct).GPIO_Mode = GPIO_Mode_IN;
    (*GPIO_InitStruct).GPIO_Speed = GPIO_Speed_Level_2;
    (*GPIO_InitStruct).GPIO_OType = GPIO_OType_PP;
    (*GPIO_InitStruct).GPIO_PuPd = GPIO_PuPd_NOPULL;
}
/* *
  * @brief  Locks GPIO Pins configuration registers.
  *         The locked registers are GPIOx_MODER, GPIOx_OTYPER, GPIOx_OSPEEDR,
  *         GPIOx_PUPDR, GPIOx_AFRL and GPIOx_AFRH.
  * @note   The configuration of the locked GPIO pins can no longer be modified
  *         until the next reset.
  * @param  GPIOx: where x can be (A or B or D) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_PinLockConfig(mut GPIOx: *mut GPIO_TypeDef,
                                            mut GPIO_Pin: uint16_t) {
    let mut tmp: uint32_t = 0x10000 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp |= GPIO_Pin as libc::c_uint;
    /* Set LCKK bit */
    ::core::ptr::write_volatile(&mut (*GPIOx).LCKR as *mut uint32_t, tmp);
    /* Reset LCKK bit */
    ::core::ptr::write_volatile(&mut (*GPIOx).LCKR as *mut uint32_t,
                                GPIO_Pin as uint32_t);
    /* Set LCKK bit */
    ::core::ptr::write_volatile(&mut (*GPIOx).LCKR as *mut uint32_t, tmp);
    /* Read LCKK bit */
    tmp = (*GPIOx).LCKR;
    /* Read LCKK bit */
    tmp = (*GPIOx).LCKR;
}
/* *
  * @}
  */
/* * @defgroup GPIO_Group2 GPIO Read and Write
 *  @brief    GPIO Read and Write
 *
@verbatim
 ===============================================================================
                  ##### GPIO Read and Write #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to read.
  * @note   This parameter can be GPIO_Pin_x where x can be :
  *         (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE;
  *         (0..2, 4, 6, 9..10) for GPIOF.
  * @retval The input port pin value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadInputDataBit(mut GPIOx: *mut GPIO_TypeDef,
                                               mut GPIO_Pin: uint16_t)
 -> uint8_t {
    let mut bitstatus: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    if ((*GPIOx).IDR as libc::c_int & GPIO_Pin as libc::c_int) as libc::c_uint
           != Bit_RESET as libc::c_int as uint32_t {
        bitstatus = Bit_SET as libc::c_int as uint8_t
    } else { bitstatus = Bit_RESET as libc::c_int as uint8_t }
    return bitstatus;
}
/* *
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @retval The input port pin value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadInputData(mut GPIOx: *mut GPIO_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    return (*GPIOx).IDR;
}
/* *
  * @brief  Reads the specified output data port bit.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_Pin: Specifies the port bit to read.
  * @note   This parameter can be GPIO_Pin_x where x can be :
  *         (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE;
  *         (0..2, 4, 6, 9..10) for GPIOF.
  * @retval The output port pin value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadOutputDataBit(mut GPIOx: *mut GPIO_TypeDef,
                                                mut GPIO_Pin: uint16_t)
 -> uint8_t {
    let mut bitstatus: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    if ((*GPIOx).ODR as libc::c_int & GPIO_Pin as libc::c_int) as libc::c_uint
           != Bit_RESET as libc::c_int as uint32_t {
        bitstatus = Bit_SET as libc::c_int as uint8_t
    } else { bitstatus = Bit_RESET as libc::c_int as uint8_t }
    return bitstatus;
}
/* *
  * @brief  Reads the specified GPIO output data port.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadOutputData(mut GPIOx: *mut GPIO_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    return (*GPIOx).ODR;
}
/* *
  * @brief  Sets the selected data port bits.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  * @note   This parameter can be GPIO_Pin_x where x can be :
  *         (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE;
  *         (0..2, 4, 6, 9..10) for GPIOF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_SetBits(mut GPIOx: *mut GPIO_TypeDef,
                                      mut GPIO_Pin: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as *mut uint32_t,
                                GPIO_Pin as uint32_t);
}
/* *
  * @brief  Clears the selected data port bits.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  * @note   This parameter can be GPIO_Pin_x where x can be :
  *         (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE;
  *         (0..2, 4, 6, 9..10) for GPIOF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ResetBits(mut GPIOx: *mut GPIO_TypeDef,
                                        mut GPIO_Pin: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*GPIOx).BRR as *mut uint16_t, GPIO_Pin);
}
/* *
  * @brief  Sets or clears the selected data port bit.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  * @note   This parameter can be GPIO_Pin_x where x can be :
  *         (0..15) for GPIOA, GPIOB, GPIOC, GPIOD or GPIOE;
  *         (0..2, 4, 6, 9..10) for GPIOF.
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enumeration values:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_WriteBit(mut GPIOx: *mut GPIO_TypeDef,
                                       mut GPIO_Pin: uint16_t,
                                       mut BitVal: BitAction) {
    /* Check the parameters */
    if BitVal as libc::c_uint != Bit_RESET as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as *mut uint32_t,
                                    GPIO_Pin as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*GPIOx).BRR as *mut uint16_t,
                                    GPIO_Pin)
    };
}
/* *
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data 
  *                  register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_Write(mut GPIOx: *mut GPIO_TypeDef,
                                    mut PortVal: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*GPIOx).ODR as *mut uint16_t, PortVal);
}
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the GPIO 
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
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup Configuration_Mode_enumeration 
  * @{
  */
/* !< GPIO Input Mode */
/* !< GPIO Output Mode */
/* !< GPIO Alternate function Mode */
/* !< GPIO Analog In/Out Mode      */
/* *
  * @}
  */
/* * @defgroup Output_type_enumeration
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
/* !< Fast Speed     */
/* !< Meduim Speed   */
/* !< High Speed     */
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Bit_SET_and_Bit_RESET_enumeration
  * @{
  */
/* *
  * @}
  */
/* * 
  * @brief  GPIO Init structure definition  
  */
/* !< Specifies the GPIO pins to be configured.
                                       This parameter can be any value of @ref GPIO_pins_define */
/* !< Specifies the operating mode for the selected pins.
                                       This parameter can be a value of @ref GPIOMode_TypeDef   */
/* !< Specifies the speed for the selected pins.
                                       This parameter can be a value of @ref GPIOSpeed_TypeDef  */
/* !< Specifies the operating output type for the selected pins.
                                       This parameter can be a value of @ref GPIOOType_TypeDef  */
/* !< Specifies the operating Pull-up/Pull down for the selected pins.
                                       This parameter can be a value of @ref GPIOPuPd_TypeDef   */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup GPIO_Exported_Constants
  * @{
  */
/* * @defgroup GPIO_pins_define 
  * @{
  */
/* !< Pin 0 selected    */
/* !< Pin 1 selected    */
/* !< Pin 2 selected    */
/* !< Pin 3 selected    */
/* !< Pin 4 selected    */
/* !< Pin 5 selected    */
/* !< Pin 6 selected    */
/* !< Pin 7 selected    */
/* !< Pin 8 selected    */
/* !< Pin 9 selected    */
/* !< Pin 10 selected   */
/* !< Pin 11 selected   */
/* !< Pin 12 selected   */
/* !< Pin 13 selected   */
/* !< Pin 14 selected   */
/* !< Pin 15 selected   */
/* !< All pins selected */
/* *
  * @}
  */
/* * @defgroup GPIO_Pin_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Alternate_function_selection_define 
  * @{
  */
/* * 
  * @brief  AF 0 selection
  */
/* JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT,  
                                                MCO, NJTRST, TRACED, TRACECK */
/* * 
  * @brief  AF 1 selection
  */
/*  OUT, TIM2, TIM15, TIM16, TIM17 */
/* * 
  * @brief  AF 2 selection
  */
/* COMP1_OUT, TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16 */
/* * 
  * @brief  AF 3 selection
  */
/* COMP7_OUT, TIM8, TIM15, Touch, HRTIM1 */
/* * 
  * @brief  AF 4 selection
  */
/* I2C1, I2C2, TIM1, TIM8, TIM16, TIM17 */
/* * 
  * @brief  AF 5 selection
  */
/* IR_OUT, I2S2, I2S3, SPI1, SPI2, TIM8, USART4, USART5 */
/* * 
  * @brief  AF 6 selection
  */
/*  IR_OUT, I2S2, I2S3, SPI2, SPI3, TIM1, TIM8 */
/* * 
  * @brief  AF 7 selection
  */
/* AOP2_OUT, CAN, COMP3_OUT, COMP5_OUT, COMP6_OUT, 
                                                USART1, USART2, USART3 */
/* * 
  * @brief  AF 8 selection
  */
/* COMP1_OUT, COMP2_OUT, COMP3_OUT, COMP4_OUT, 
                                                COMP5_OUT, COMP6_OUT */
/* * 
  * @brief  AF 9 selection
  */
/* AOP4_OUT, CAN, TIM1, TIM8, TIM15 */
/* * 
  * @brief  AF 10 selection
  */
/* AOP1_OUT, AOP3_OUT, TIM2, TIM3, TIM4, TIM8, TIM17 */
/* * 
  * @brief  AF 11 selection
  */
/* TIM1, TIM8 */
/* * 
   * @brief  AF 12 selection
   */
/* TIM1, HRTIM1 */
/* * 
   * @brief  AF 13 selection
   */
/* HRTIM1, AOP2_OUT */
/* * 
  * @brief  AF 14 selection
  */
/* USBDM, USBDP */
/* * 
  * @brief  AF 15 selection
  */
/* OUT */
/* *
  * @}
  */
/* * @defgroup GPIO_Speed_Legacy 
  * @{
  */
/* !< Fast Speed:10MHz   */
/* !< Medium Speed:2MHz  */
/* !< High Speed:50MHz   */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 
/* Function used to set the GPIO configuration to the default reset state *****/
/* Initialization and Configuration functions *********************************/
/* GPIO Read and Write functions **********************************************/
/* GPIO Alternate functions configuration functions ***************************/
/* *
  * @}
  */
/* * @defgroup GPIO_Group3 GPIO Alternate functions configuration functions
 *  @brief   GPIO Alternate functions configuration functions
 *
@verbatim
 ===============================================================================
          ##### GPIO Alternate functions configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A, B, C, D, E or F) to select the GPIO peripheral.
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @param  GPIO_AF: selects the pin to be used as Alternate function.  
  *   This parameter can be one of the following value:
  *     @arg GPIO_AF_0:  JTCK-SWCLK, JTDI, JTDO/TRACESW0, JTMS-SWDAT, MCO, NJTRST, 
  *                      TRACED, TRACECK.
  *     @arg GPIO_AF_1:  OUT, TIM2, TIM15, TIM16, TIM17.
  *     @arg GPIO_AF_2:  COMP1_OUT, TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16.
  *     @arg GPIO_AF_3:  COMP7_OUT, TIM8, TIM15, Touch, HRTIM.
  *     @arg GPIO_AF_4:  I2C1, I2C2, TIM1, TIM8, TIM16, TIM17.
  *     @arg GPIO_AF_5:  IR_OUT, I2S2, I2S3, SPI1, SPI2, TIM8, USART4, USART5
  *     @arg GPIO_AF_6:  IR_OUT, I2S2, I2S3, SPI2, SPI3, TIM1, TIM8
  *     @arg GPIO_AF_7:  AOP2_OUT, CAN, COMP3_OUT, COMP5_OUT, COMP6_OUT, USART1, 
  *                      USART2, USART3.
  *     @arg GPIO_AF_8:  COMP1_OUT, COMP2_OUT, COMP3_OUT, COMP4_OUT, COMP5_OUT, 
  *                      COMP6_OUT.
  *     @arg GPIO_AF_9:  AOP4_OUT, CAN, TIM1, TIM8, TIM15.
  *     @arg GPIO_AF_10: AOP1_OUT, AOP3_OUT, TIM2, TIM3, TIM4, TIM8, TIM17. 
  *     @arg GPIO_AF_11: TIM1, TIM8.
  *     @arg GPIO_AF_12: TIM1, HRTIM.
  *     @arg GPIO_AF_13: HRTIM, AOP2_OUT.
  *     @arg GPIO_AF_14: USBDM, USBDP.
  *     @arg GPIO_AF_15: OUT.             
  * @note  The pin should already been configured in Alternate Function mode(AF)
  *        using GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
  * @note  Refer to the Alternate function mapping table in the device datasheet 
  *        for the detailed mapping of the system and peripherals alternate 
  *        function I/O pins.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_PinAFConfig(mut GPIOx: *mut GPIO_TypeDef,
                                          mut GPIO_PinSource: uint16_t,
                                          mut GPIO_AF: uint8_t) {
    let mut temp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut temp_2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    temp =
        (GPIO_AF as uint32_t) <<
            (GPIO_PinSource as uint32_t &
                 0x7 as libc::c_int as
                     uint32_t).wrapping_mul(4 as libc::c_int as libc::c_uint);
    ::core::ptr::write_volatile(&mut (*GPIOx).AFR[(GPIO_PinSource as
                                                       libc::c_int >>
                                                       0x3 as libc::c_int) as
                                                      usize] as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*GPIOx).AFR[(GPIO_PinSource
                                                                                           as
                                                                                           libc::c_int
                                                                                           >>
                                                                                           0x3
                                                                                               as
                                                                                               libc::c_int)
                                                                                          as
                                                                                          usize]
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0xf as libc::c_int as uint32_t) <<
                                           (GPIO_PinSource as uint32_t &
                                                0x7 as libc::c_int as
                                                    uint32_t).wrapping_mul(4
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint)))
                                    as uint32_t as uint32_t);
    temp_2 =
        (*GPIOx).AFR[(GPIO_PinSource as libc::c_int >> 0x3 as libc::c_int) as
                         usize] | temp;
    ::core::ptr::write_volatile(&mut (*GPIOx).AFR[(GPIO_PinSource as
                                                       libc::c_int >>
                                                       0x3 as libc::c_int) as
                                                      usize] as *mut uint32_t,
                                temp_2);
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
