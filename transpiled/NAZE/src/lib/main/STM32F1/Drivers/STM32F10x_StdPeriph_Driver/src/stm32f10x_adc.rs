use ::libc;
extern "C" {
    /* STM32F10X_CL */
    #[no_mangle]
    fn RCC_APB2PeriphResetCmd(RCC_APB2Periph: uint32_t,
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
/* !< STM32F10x Standard Peripheral Library old definitions (maintained for legacy purpose) */
/* *
  * @}
  */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* * 
  * @brief Analog to Digital Converter  
  */
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
/* *
  ******************************************************************************
  * @file    stm32f10x_adc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the ADC firmware 
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
/* * @addtogroup ADC
  * @{
  */
/* * @defgroup ADC_Exported_Types
  * @{
  */
/* * 
  * @brief  ADC Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_InitTypeDef {
    pub ADC_Mode: uint32_t,
    pub ADC_ScanConvMode: FunctionalState,
    pub ADC_ContinuousConvMode: FunctionalState,
    pub ADC_ExternalTrigConv: uint32_t,
    pub ADC_DataAlign: uint32_t,
    pub ADC_NbrOfChannel: uint8_t,
}
/* *
  * @}
  */
/* * @defgroup ADC_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the ADCx peripheral registers to their default reset values.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DeInit(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    if ADCx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x2400
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef {
        /* Enable ADC1 reset state */
        RCC_APB2PeriphResetCmd(0x200 as libc::c_int as uint32_t, ENABLE);
        /* Release ADC1 from reset state */
        RCC_APB2PeriphResetCmd(0x200 as libc::c_int as uint32_t, DISABLE);
    } else if ADCx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x2800
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut ADC_TypeDef {
        /* Enable ADC2 reset state */
        RCC_APB2PeriphResetCmd(0x400 as libc::c_int as uint32_t, ENABLE);
        /* Release ADC2 from reset state */
        RCC_APB2PeriphResetCmd(0x400 as libc::c_int as uint32_t, DISABLE);
    } else if ADCx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x3c00
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut ADC_TypeDef {
        /* Enable ADC3 reset state */
        RCC_APB2PeriphResetCmd(0x8000 as libc::c_int as uint32_t, ENABLE);
        /* Release ADC3 from reset state */
        RCC_APB2PeriphResetCmd(0x8000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_InitStruct: pointer to an ADC_InitTypeDef structure that contains
  *         the configuration information for the specified ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_Init(mut ADCx: *mut ADC_TypeDef,
                                  mut ADC_InitStruct: *mut ADC_InitTypeDef) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    /*---------------------------- ADCx CR1 Configuration -----------------*/
  /* Get the ADCx CR1 value */
    tmpreg1 = (*ADCx).CR1;
    /* Clear DUALMOD and SCAN bits */
    tmpreg1 &= 0xfff0feff as libc::c_uint;
    /* Configure ADCx: Dual mode and scan conversion mode */
  /* Set DUALMOD bits according to ADC_Mode value */
  /* Set SCAN bit according to ADC_ScanConvMode value */
    tmpreg1 |=
        (*ADC_InitStruct).ADC_Mode |
            ((*ADC_InitStruct).ADC_ScanConvMode as uint32_t) <<
                8 as libc::c_int;
    /* Write to ADCx CR1 */
    ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t, tmpreg1);
    /*---------------------------- ADCx CR2 Configuration -----------------*/
  /* Get the ADCx CR2 value */
    tmpreg1 = (*ADCx).CR2;
    /* Clear CONT, ALIGN and EXTSEL bits */
    tmpreg1 &= 0xfff1f7fd as libc::c_uint;
    /* Configure ADCx: external trigger event and continuous conversion mode */
  /* Set ALIGN bit according to ADC_DataAlign value */
  /* Set EXTSEL bits according to ADC_ExternalTrigConv value */
  /* Set CONT bit according to ADC_ContinuousConvMode value */
    tmpreg1 |=
        (*ADC_InitStruct).ADC_DataAlign |
            (*ADC_InitStruct).ADC_ExternalTrigConv |
            ((*ADC_InitStruct).ADC_ContinuousConvMode as uint32_t) <<
                1 as libc::c_int;
    /* Write to ADCx CR2 */
    ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t, tmpreg1);
    /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
    tmpreg1 = (*ADCx).SQR1;
    /* Clear L bits */
    tmpreg1 &= 0xff0fffff as libc::c_uint;
    /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfChannel value */
    tmpreg2 =
        (tmpreg2 as libc::c_int |
             ((*ADC_InitStruct).ADC_NbrOfChannel as libc::c_int -
                  1 as libc::c_int as uint8_t as libc::c_int) as uint8_t as
                 libc::c_int) as uint8_t;
    tmpreg1 |= (tmpreg2 as uint32_t) << 20 as libc::c_int;
    /* Write to ADCx SQR1 */
    ::core::ptr::write_volatile(&mut (*ADCx).SQR1 as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Fills each ADC_InitStruct member with its default value.
  * @param  ADC_InitStruct : pointer to an ADC_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StructInit(mut ADC_InitStruct:
                                            *mut ADC_InitTypeDef) {
    /* Reset ADC init structure parameters values */
  /* Initialize the ADC_Mode member */
    (*ADC_InitStruct).ADC_Mode = 0 as libc::c_int as uint32_t;
    /* initialize the ADC_ScanConvMode member */
    (*ADC_InitStruct).ADC_ScanConvMode = DISABLE;
    /* Initialize the ADC_ContinuousConvMode member */
    (*ADC_InitStruct).ADC_ContinuousConvMode = DISABLE;
    /* Initialize the ADC_ExternalTrigConv member */
    (*ADC_InitStruct).ADC_ExternalTrigConv = 0 as libc::c_int as uint32_t;
    /* Initialize the ADC_DataAlign member */
    (*ADC_InitStruct).ADC_DataAlign = 0 as libc::c_int as uint32_t;
    /* Initialize the ADC_NbrOfChannel member */
    (*ADC_InitStruct).ADC_NbrOfChannel = 1 as libc::c_int as uint8_t;
}
/* *
  * @brief  Enables or disables the specified ADC peripheral.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_Cmd(mut ADCx: *mut ADC_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the ADON bit to wake up the ADC from power down mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC peripheral */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffffffe as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the specified ADC DMA request.
  * @param  ADCx: where x can be 1 or 3 to select the ADC peripheral.
  *   Note: ADC2 hasn't a DMA capability.
  * @param  NewState: new state of the selected ADC DMA transfer.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DMACmd(mut ADCx: *mut ADC_TypeDef,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC DMA request */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x100 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC DMA request */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffffeff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the specified ADC interrupts.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_IT: specifies the ADC interrupt sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg ADC_IT_EOC: End of conversion interrupt mask
  *     @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *     @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  * @param  NewState: new state of the specified ADC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ITConfig(mut ADCx: *mut ADC_TypeDef,
                                      mut ADC_IT: uint16_t,
                                      mut NewState: FunctionalState) {
    let mut itmask: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    /* Get the ADC IT index */
    itmask = ADC_IT as uint8_t;
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC interrupts */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         itmask as libc::c_uint) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC interrupts */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(itmask as uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Resets the selected ADC calibration registers.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ResetCalibration(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Resets the selected ADC calibration registers */
    ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x8 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the selected ADC reset calibration registers status.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval The new state of ADC reset calibration registers (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetResetCalibrationStatus(mut ADCx:
                                                           *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of RSTCAL bit */
    if (*ADCx).CR2 & 0x8 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* RSTCAL bit is set */
        bitstatus = SET
    } else {
        /* RSTCAL bit is reset */
        bitstatus = RESET
    }
    /* Return the RSTCAL bit status */
    return bitstatus;
}
/* *
  * @brief  Starts the selected ADC calibration process.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StartCalibration(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Enable the selected ADC calibration process */
    ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x4 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the selected ADC calibration status.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval The new state of ADC calibration (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetCalibrationStatus(mut ADCx: *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of CAL bit */
    if (*ADCx).CR2 & 0x4 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* CAL bit is set: calibration on going */
        bitstatus = SET
    } else {
        /* CAL bit is reset: end of calibration */
        bitstatus = RESET
    }
    /* Return the CAL bit status */
    return bitstatus;
}
/* *
  * @brief  Enables or disables the selected ADC software start conversion .
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC software start conversion.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SoftwareStartConvCmd(mut ADCx: *mut ADC_TypeDef,
                                                  mut NewState:
                                                      FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC conversion on external event and start the selected
       ADC conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x500000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC conversion on external event and stop the selected
       ADC conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffafffff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Gets the selected ADC Software start conversion Status.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval The new state of ADC software start conversion (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetSoftwareStartConvStatus(mut ADCx:
                                                            *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of SWSTART bit */
    if (*ADCx).CR2 & 0x400000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* SWSTART bit is set */
        bitstatus = SET
    } else {
        /* SWSTART bit is reset */
        bitstatus = RESET
    }
    /* Return the SWSTART bit status */
    return bitstatus;
}
/* *
  * @brief  Configures the discontinuous mode for the selected ADC regular
  *         group channel.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  Number: specifies the discontinuous mode regular channel
  *         count value. This number must be between 1 and 8.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DiscModeChannelCountConfig(mut ADCx:
                                                            *mut ADC_TypeDef,
                                                        mut Number: uint8_t) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg1 = (*ADCx).CR1;
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= 0xffff1fff as libc::c_uint;
    /* Set the discontinuous mode channel count */
    tmpreg2 = (Number as libc::c_int - 1 as libc::c_int) as uint32_t;
    tmpreg1 |= tmpreg2 << 13 as libc::c_int;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Enables or disables the discontinuous mode on regular group
  *         channel for the specified ADC
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC discontinuous mode
  *         on regular group channel.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DiscModeCmd(mut ADCx: *mut ADC_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC regular discontinuous mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC regular discontinuous mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffff7ff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures for the selected ADC regular channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
  *     @arg ADC_Channel_0: ADC Channel0 selected
  *     @arg ADC_Channel_1: ADC Channel1 selected
  *     @arg ADC_Channel_2: ADC Channel2 selected
  *     @arg ADC_Channel_3: ADC Channel3 selected
  *     @arg ADC_Channel_4: ADC Channel4 selected
  *     @arg ADC_Channel_5: ADC Channel5 selected
  *     @arg ADC_Channel_6: ADC Channel6 selected
  *     @arg ADC_Channel_7: ADC Channel7 selected
  *     @arg ADC_Channel_8: ADC Channel8 selected
  *     @arg ADC_Channel_9: ADC Channel9 selected
  *     @arg ADC_Channel_10: ADC Channel10 selected
  *     @arg ADC_Channel_11: ADC Channel11 selected
  *     @arg ADC_Channel_12: ADC Channel12 selected
  *     @arg ADC_Channel_13: ADC Channel13 selected
  *     @arg ADC_Channel_14: ADC Channel14 selected
  *     @arg ADC_Channel_15: ADC Channel15 selected
  *     @arg ADC_Channel_16: ADC Channel16 selected
  *     @arg ADC_Channel_17: ADC Channel17 selected
  * @param  Rank: The rank in the regular group sequencer. This parameter must be between 1 to 16.
  * @param  ADC_SampleTime: The sample time value to be set for the selected channel. 
  *   This parameter can be one of the following values:
  *     @arg ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
  *     @arg ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles
  *     @arg ADC_SampleTime_13Cycles5: Sample time equal to 13.5 cycles
  *     @arg ADC_SampleTime_28Cycles5: Sample time equal to 28.5 cycles	
  *     @arg ADC_SampleTime_41Cycles5: Sample time equal to 41.5 cycles	
  *     @arg ADC_SampleTime_55Cycles5: Sample time equal to 55.5 cycles	
  *     @arg ADC_SampleTime_71Cycles5: Sample time equal to 71.5 cycles	
  *     @arg ADC_SampleTime_239Cycles5: Sample time equal to 239.5 cycles	
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_RegularChannelConfig(mut ADCx: *mut ADC_TypeDef,
                                                  mut ADC_Channel: uint8_t,
                                                  mut Rank: uint8_t,
                                                  mut ADC_SampleTime:
                                                      uint8_t) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* if ADC_Channel_10 ... ADC_Channel_17 is selected */
    if ADC_Channel as libc::c_int >
           0x9 as libc::c_int as uint8_t as libc::c_int {
        /* Get the old register value */
        tmpreg1 = (*ADCx).SMPR1;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x7 as libc::c_int as uint32_t) <<
                3 as libc::c_int *
                    (ADC_Channel as libc::c_int - 10 as libc::c_int);
        /* Clear the old channel sample time */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_SampleTime as uint32_t) <<
                3 as libc::c_int *
                    (ADC_Channel as libc::c_int - 10 as libc::c_int);
        /* Set the new channel sample time */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR1 as *mut uint32_t,
                                    tmpreg1)
    } else {
        /* ADC_Channel include in ADC_Channel_[0..9] */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SMPR2;
        tmpreg2 =
            (0x7 as libc::c_int as uint32_t) <<
                3 as libc::c_int * ADC_Channel as libc::c_int;
        tmpreg1 &= !tmpreg2;
        tmpreg2 =
            (ADC_SampleTime as uint32_t) <<
                3 as libc::c_int * ADC_Channel as libc::c_int;
        tmpreg1 |= tmpreg2;
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR2 as *mut uint32_t,
                                    tmpreg1)
    }
    /* Calculate the mask to clear */
    /* Clear the old channel sample time */
    /* Calculate the mask to set */
    /* Set the new channel sample time */
    /* Store the new register value */
    /* For Rank 1 to 6 */
    if (Rank as libc::c_int) < 7 as libc::c_int {
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR3;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x1f as libc::c_int as uint32_t) <<
                5 as libc::c_int * (Rank as libc::c_int - 1 as libc::c_int);
        /* Clear the old SQx bits for the selected rank */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_Channel as uint32_t) <<
                5 as libc::c_int * (Rank as libc::c_int - 1 as libc::c_int);
        /* Set the SQx bits for the selected rank */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SQR3 as *mut uint32_t,
                                    tmpreg1)
    } else if (Rank as libc::c_int) < 13 as libc::c_int {
        /* For Rank 7 to 12 */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR2;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x1f as libc::c_int as uint32_t) <<
                5 as libc::c_int * (Rank as libc::c_int - 7 as libc::c_int);
        /* Clear the old SQx bits for the selected rank */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_Channel as uint32_t) <<
                5 as libc::c_int * (Rank as libc::c_int - 7 as libc::c_int);
        /* Set the SQx bits for the selected rank */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SQR2 as *mut uint32_t,
                                    tmpreg1)
    } else {
        /* For Rank 13 to 16 */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR1;
        tmpreg2 =
            (0x1f as libc::c_int as uint32_t) <<
                5 as libc::c_int * (Rank as libc::c_int - 13 as libc::c_int);
        tmpreg1 &= !tmpreg2;
        tmpreg2 =
            (ADC_Channel as uint32_t) <<
                5 as libc::c_int * (Rank as libc::c_int - 13 as libc::c_int);
        tmpreg1 |= tmpreg2;
        ::core::ptr::write_volatile(&mut (*ADCx).SQR1 as *mut uint32_t,
                                    tmpreg1)
    };
}
/* Calculate the mask to clear */
/* Clear the old SQx bits for the selected rank */
/* Calculate the mask to set */
/* Set the SQx bits for the selected rank */
/* Store the new register value */
/* *
  * @brief  Enables or disables the ADCx conversion through external trigger.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC external trigger start of conversion.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ExternalTrigConvCmd(mut ADCx: *mut ADC_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC conversion on external event */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x100000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC conversion on external event */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffefffff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Returns the last ADCx conversion result data for regular channel.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval The Data conversion value.
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetConversionValue(mut ADCx: *mut ADC_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Return the selected ADC conversion value */
    return (*ADCx).DR as uint16_t;
}
/* *
  * @brief  Returns the last ADC1 and ADC2 conversion result data in dual mode.
  * @retval The Data conversion value.
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetDualModeConversionValue() -> uint32_t {
    /* Return the dual mode conversion value */
    return *(0x4001244c as libc::c_int as uint32_t as *mut uint32_t);
}
/* *
  * @brief  Enables or disables the selected ADC automatic injected group
  *         conversion after regular one.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC auto injected conversion
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AutoInjectedConvCmd(mut ADCx: *mut ADC_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC automatic injected group conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x400 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC automatic injected group conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffffbff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the discontinuous mode for injected group
  *         channel for the specified ADC
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC discontinuous mode
  *         on injected group channel.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_InjectedDiscModeCmd(mut ADCx: *mut ADC_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC injected discontinuous mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC injected discontinuous mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffffefff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures the ADCx external trigger for injected channels conversion.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_ExternalTrigInjecConv: specifies the ADC trigger to start injected conversion. 
  *   This parameter can be one of the following values:
  *     @arg ADC_ExternalTrigInjecConv_T1_TRGO: Timer1 TRGO event selected (for ADC1, ADC2 and ADC3)
  *     @arg ADC_ExternalTrigInjecConv_T1_CC4: Timer1 capture compare4 selected (for ADC1, ADC2 and ADC3)
  *     @arg ADC_ExternalTrigInjecConv_T2_TRGO: Timer2 TRGO event selected (for ADC1 and ADC2)
  *     @arg ADC_ExternalTrigInjecConv_T2_CC1: Timer2 capture compare1 selected (for ADC1 and ADC2)
  *     @arg ADC_ExternalTrigInjecConv_T3_CC4: Timer3 capture compare4 selected (for ADC1 and ADC2)
  *     @arg ADC_ExternalTrigInjecConv_T4_TRGO: Timer4 TRGO event selected (for ADC1 and ADC2)
  *     @arg ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4: External interrupt line 15 or Timer8
  *                                                       capture compare4 event selected (for ADC1 and ADC2)                       
  *     @arg ADC_ExternalTrigInjecConv_T4_CC3: Timer4 capture compare3 selected (for ADC3 only)
  *     @arg ADC_ExternalTrigInjecConv_T8_CC2: Timer8 capture compare2 selected (for ADC3 only)                         
  *     @arg ADC_ExternalTrigInjecConv_T8_CC4: Timer8 capture compare4 selected (for ADC3 only)
  *     @arg ADC_ExternalTrigInjecConv_T5_TRGO: Timer5 TRGO event selected (for ADC3 only)                         
  *     @arg ADC_ExternalTrigInjecConv_T5_CC4: Timer5 capture compare4 selected (for ADC3 only)                        
  *     @arg ADC_ExternalTrigInjecConv_None: Injected conversion started by software and not
  *                                          by external trigger (for ADC1, ADC2 and ADC3)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ExternalTrigInjectedConvConfig(mut ADCx:
                                                                *mut ADC_TypeDef,
                                                            mut ADC_ExternalTrigInjecConv:
                                                                uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).CR2;
    /* Clear the old external event selection for injected group */
    tmpreg &= 0xffff8fff as libc::c_uint;
    /* Set the external event selection for injected group */
    tmpreg |= ADC_ExternalTrigInjecConv;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the ADCx injected channels conversion through
  *         external trigger
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC external trigger start of
  *         injected conversion.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ExternalTrigInjectedConvCmd(mut ADCx:
                                                             *mut ADC_TypeDef,
                                                         mut NewState:
                                                             FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC external event selection for injected group */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC external event selection for injected group */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffff7fff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the selected ADC start of the injected 
  *         channels conversion.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC software start injected conversion.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SoftwareStartInjectedConvCmd(mut ADCx:
                                                              *mut ADC_TypeDef,
                                                          mut NewState:
                                                              FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC conversion for injected group on external event and start the selected
       ADC injected conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x208000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC conversion on external event for injected group and stop the selected
       ADC injected conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xffdf7fff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Gets the selected ADC Software start injected conversion Status.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @retval The new state of ADC software start injected conversion (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetSoftwareStartInjectedConvCmdStatus(mut ADCx:
                                                                       *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of JSWSTART bit */
    if (*ADCx).CR2 & 0x200000 as libc::c_int as uint32_t !=
           RESET as libc::c_int as uint32_t {
        /* JSWSTART bit is set */
        bitstatus = SET
    } else {
        /* JSWSTART bit is reset */
        bitstatus = RESET
    }
    /* Return the JSWSTART bit status */
    return bitstatus;
}
/* *
  * @brief  Configures for the selected ADC injected channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
  *     @arg ADC_Channel_0: ADC Channel0 selected
  *     @arg ADC_Channel_1: ADC Channel1 selected
  *     @arg ADC_Channel_2: ADC Channel2 selected
  *     @arg ADC_Channel_3: ADC Channel3 selected
  *     @arg ADC_Channel_4: ADC Channel4 selected
  *     @arg ADC_Channel_5: ADC Channel5 selected
  *     @arg ADC_Channel_6: ADC Channel6 selected
  *     @arg ADC_Channel_7: ADC Channel7 selected
  *     @arg ADC_Channel_8: ADC Channel8 selected
  *     @arg ADC_Channel_9: ADC Channel9 selected
  *     @arg ADC_Channel_10: ADC Channel10 selected
  *     @arg ADC_Channel_11: ADC Channel11 selected
  *     @arg ADC_Channel_12: ADC Channel12 selected
  *     @arg ADC_Channel_13: ADC Channel13 selected
  *     @arg ADC_Channel_14: ADC Channel14 selected
  *     @arg ADC_Channel_15: ADC Channel15 selected
  *     @arg ADC_Channel_16: ADC Channel16 selected
  *     @arg ADC_Channel_17: ADC Channel17 selected
  * @param  Rank: The rank in the injected group sequencer. This parameter must be between 1 and 4.
  * @param  ADC_SampleTime: The sample time value to be set for the selected channel. 
  *   This parameter can be one of the following values:
  *     @arg ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
  *     @arg ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles
  *     @arg ADC_SampleTime_13Cycles5: Sample time equal to 13.5 cycles
  *     @arg ADC_SampleTime_28Cycles5: Sample time equal to 28.5 cycles	
  *     @arg ADC_SampleTime_41Cycles5: Sample time equal to 41.5 cycles	
  *     @arg ADC_SampleTime_55Cycles5: Sample time equal to 55.5 cycles	
  *     @arg ADC_SampleTime_71Cycles5: Sample time equal to 71.5 cycles	
  *     @arg ADC_SampleTime_239Cycles5: Sample time equal to 239.5 cycles	
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_InjectedChannelConfig(mut ADCx: *mut ADC_TypeDef,
                                                   mut ADC_Channel: uint8_t,
                                                   mut Rank: uint8_t,
                                                   mut ADC_SampleTime:
                                                       uint8_t) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg3: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* if ADC_Channel_10 ... ADC_Channel_17 is selected */
    if ADC_Channel as libc::c_int >
           0x9 as libc::c_int as uint8_t as libc::c_int {
        /* Get the old register value */
        tmpreg1 = (*ADCx).SMPR1;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x7 as libc::c_int as uint32_t) <<
                3 as libc::c_int *
                    (ADC_Channel as libc::c_int - 10 as libc::c_int);
        /* Clear the old channel sample time */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_SampleTime as uint32_t) <<
                3 as libc::c_int *
                    (ADC_Channel as libc::c_int - 10 as libc::c_int);
        /* Set the new channel sample time */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR1 as *mut uint32_t,
                                    tmpreg1)
    } else {
        /* ADC_Channel include in ADC_Channel_[0..9] */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SMPR2;
        tmpreg2 =
            (0x7 as libc::c_int as uint32_t) <<
                3 as libc::c_int * ADC_Channel as libc::c_int;
        tmpreg1 &= !tmpreg2;
        tmpreg2 =
            (ADC_SampleTime as uint32_t) <<
                3 as libc::c_int * ADC_Channel as libc::c_int;
        tmpreg1 |= tmpreg2;
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR2 as *mut uint32_t,
                                    tmpreg1)
    }
    /* Calculate the mask to clear */
    /* Clear the old channel sample time */
    /* Calculate the mask to set */
    /* Set the new channel sample time */
    /* Store the new register value */
    /* Rank configuration */
  /* Get the old register value */
    tmpreg1 = (*ADCx).JSQR;
    /* Get JL value: Number = JL+1 */
    tmpreg3 =
        (tmpreg1 & 0x300000 as libc::c_int as uint32_t) >> 20 as libc::c_int;
    /* Calculate the mask to clear: ((Rank-1)+(4-JL-1)) */
    tmpreg2 =
        (0x1f as libc::c_int as uint32_t) <<
            5 as libc::c_int *
                ((Rank as libc::c_int + 3 as libc::c_int) as
                     libc::c_uint).wrapping_sub(tmpreg3.wrapping_add(1 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint))
                    as uint8_t as libc::c_int;
    /* Clear the old JSQx bits for the selected rank */
    tmpreg1 &= !tmpreg2;
    /* Calculate the mask to set: ((Rank-1)+(4-JL-1)) */
    tmpreg2 =
        (ADC_Channel as uint32_t) <<
            5 as libc::c_int *
                ((Rank as libc::c_int + 3 as libc::c_int) as
                     libc::c_uint).wrapping_sub(tmpreg3.wrapping_add(1 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint))
                    as uint8_t as libc::c_int;
    /* Set the JSQx bits for the selected rank */
    tmpreg1 |= tmpreg2;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).JSQR as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Configures the sequencer length for injected channels
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  Length: The sequencer length. 
  *   This parameter must be a number between 1 to 4.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_InjectedSequencerLengthConfig(mut ADCx:
                                                               *mut ADC_TypeDef,
                                                           mut Length:
                                                               uint8_t) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg1 = (*ADCx).JSQR;
    /* Clear the old injected sequnence lenght JL bits */
    tmpreg1 &= 0xffcfffff as libc::c_uint;
    /* Set the injected sequnence lenght JL bits */
    tmpreg2 = (Length as libc::c_int - 1 as libc::c_int) as uint32_t;
    tmpreg1 |= tmpreg2 << 20 as libc::c_int;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).JSQR as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Set the injected channels conversion value offset
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_InjectedChannel: the ADC injected channel to set its offset. 
  *   This parameter can be one of the following values:
  *     @arg ADC_InjectedChannel_1: Injected Channel1 selected
  *     @arg ADC_InjectedChannel_2: Injected Channel2 selected
  *     @arg ADC_InjectedChannel_3: Injected Channel3 selected
  *     @arg ADC_InjectedChannel_4: Injected Channel4 selected
  * @param  Offset: the offset value for the selected ADC injected channel
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SetInjectedOffset(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_InjectedChannel:
                                                   uint8_t,
                                               mut Offset: uint16_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, ADCx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(ADC_InjectedChannel
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Set the selected injected channel data offset */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, Offset as uint32_t);
}
/* *
  * @brief  Returns the ADC injected channel conversion result
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_InjectedChannel: the converted ADC injected channel.
  *   This parameter can be one of the following values:
  *     @arg ADC_InjectedChannel_1: Injected Channel1 selected
  *     @arg ADC_InjectedChannel_2: Injected Channel2 selected
  *     @arg ADC_InjectedChannel_3: Injected Channel3 selected
  *     @arg ADC_InjectedChannel_4: Injected Channel4 selected
  * @retval The Data conversion value.
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetInjectedConversionValue(mut ADCx:
                                                            *mut ADC_TypeDef,
                                                        mut ADC_InjectedChannel:
                                                            uint8_t)
 -> uint16_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, ADCx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add((ADC_InjectedChannel
                                                                     as
                                                                     libc::c_int
                                                                     +
                                                                     0x28 as
                                                                         libc::c_int
                                                                         as
                                                                         uint8_t
                                                                         as
                                                                         libc::c_int)
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Returns the selected injected channel conversion data value */
    return *(tmp as *mut uint32_t) as uint16_t;
}
/* *
  * @brief  Enables or disables the analog watchdog on single/all regular
  *         or injected channels
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_AnalogWatchdog: the ADC analog watchdog configuration.
  *   This parameter can be one of the following values:
  *     @arg ADC_AnalogWatchdog_SingleRegEnable: Analog watchdog on a single regular channel
  *     @arg ADC_AnalogWatchdog_SingleInjecEnable: Analog watchdog on a single injected channel
  *     @arg ADC_AnalogWatchdog_SingleRegOrInjecEnable: Analog watchdog on a single regular or injected channel
  *     @arg ADC_AnalogWatchdog_AllRegEnable: Analog watchdog on  all regular channel
  *     @arg ADC_AnalogWatchdog_AllInjecEnable: Analog watchdog on  all injected channel
  *     @arg ADC_AnalogWatchdog_AllRegAllInjecEnable: Analog watchdog on all regular and injected channels
  *     @arg ADC_AnalogWatchdog_None: No channel guarded by the analog watchdog
  * @retval None	  
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdogCmd(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_AnalogWatchdog:
                                                   uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).CR1;
    /* Clear AWDEN, AWDENJ and AWDSGL bits */
    tmpreg &= 0xff3ffdff as libc::c_uint;
    /* Set the analog watchdog enable mode */
    tmpreg |= ADC_AnalogWatchdog;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the high and low thresholds of the analog watchdog.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  HighThreshold: the ADC analog watchdog High threshold value.
  *   This parameter must be a 12bit value.
  * @param  LowThreshold: the ADC analog watchdog Low threshold value.
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdogThresholdsConfig(mut ADCx:
                                                                *mut ADC_TypeDef,
                                                            mut HighThreshold:
                                                                uint16_t,
                                                            mut LowThreshold:
                                                                uint16_t) {
    /* Check the parameters */
    /* Set the ADCx high threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).HTR as *mut uint32_t,
                                HighThreshold as uint32_t);
    /* Set the ADCx low threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).LTR as *mut uint32_t,
                                LowThreshold as uint32_t);
}
/* *
  * @brief  Configures the analog watchdog guarded single channel
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure for the analog watchdog. 
  *   This parameter can be one of the following values:
  *     @arg ADC_Channel_0: ADC Channel0 selected
  *     @arg ADC_Channel_1: ADC Channel1 selected
  *     @arg ADC_Channel_2: ADC Channel2 selected
  *     @arg ADC_Channel_3: ADC Channel3 selected
  *     @arg ADC_Channel_4: ADC Channel4 selected
  *     @arg ADC_Channel_5: ADC Channel5 selected
  *     @arg ADC_Channel_6: ADC Channel6 selected
  *     @arg ADC_Channel_7: ADC Channel7 selected
  *     @arg ADC_Channel_8: ADC Channel8 selected
  *     @arg ADC_Channel_9: ADC Channel9 selected
  *     @arg ADC_Channel_10: ADC Channel10 selected
  *     @arg ADC_Channel_11: ADC Channel11 selected
  *     @arg ADC_Channel_12: ADC Channel12 selected
  *     @arg ADC_Channel_13: ADC Channel13 selected
  *     @arg ADC_Channel_14: ADC Channel14 selected
  *     @arg ADC_Channel_15: ADC Channel15 selected
  *     @arg ADC_Channel_16: ADC Channel16 selected
  *     @arg ADC_Channel_17: ADC Channel17 selected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdogSingleChannelConfig(mut ADCx:
                                                                   *mut ADC_TypeDef,
                                                               mut ADC_Channel:
                                                                   uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).CR1;
    /* Clear the Analog watchdog channel select bits */
    tmpreg &= 0xffffffe0 as libc::c_uint;
    /* Set the Analog watchdog channel */
    tmpreg |= ADC_Channel as libc::c_uint;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CR1 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the temperature sensor and Vrefint channel.
  * @param  NewState: new state of the temperature sensor.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_TempSensorVrefintCmd(mut NewState:
                                                      FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the temperature sensor and Vrefint channel*/
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef)).CR2;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the temperature sensor and Vrefint channel*/
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x2400
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef)).CR2;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xff7fffff as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified ADC flag is set or not.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg ADC_FLAG_AWD: Analog watchdog flag
  *     @arg ADC_FLAG_EOC: End of conversion flag
  *     @arg ADC_FLAG_JEOC: End of injected group conversion flag
  *     @arg ADC_FLAG_JSTRT: Start of injected group conversion flag
  *     @arg ADC_FLAG_STRT: Start of regular group conversion flag
  * @retval The new state of ADC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetFlagStatus(mut ADCx: *mut ADC_TypeDef,
                                           mut ADC_FLAG: uint8_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified ADC flag */
    if (*ADCx).SR & ADC_FLAG as libc::c_uint !=
           RESET as libc::c_int as uint8_t as libc::c_uint {
        /* ADC_FLAG is set */
        bitstatus = SET
    } else {
        /* ADC_FLAG is reset */
        bitstatus = RESET
    }
    /* Return the ADC_FLAG status */
    return bitstatus;
}
/* *
  * @brief  Clears the ADCx's pending flags.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_FLAG: specifies the flag to clear. 
  *   This parameter can be any combination of the following values:
  *     @arg ADC_FLAG_AWD: Analog watchdog flag
  *     @arg ADC_FLAG_EOC: End of conversion flag
  *     @arg ADC_FLAG_JEOC: End of injected group conversion flag
  *     @arg ADC_FLAG_JSTRT: Start of injected group conversion flag
  *     @arg ADC_FLAG_STRT: Start of regular group conversion flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ClearFlag(mut ADCx: *mut ADC_TypeDef,
                                       mut ADC_FLAG: uint8_t) {
    /* Check the parameters */
    /* Clear the selected ADC flags */
    ::core::ptr::write_volatile(&mut (*ADCx).SR as *mut uint32_t,
                                !(ADC_FLAG as uint32_t));
}
/* *
  * @brief  Checks whether the specified ADC interrupt has occurred or not.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_IT: specifies the ADC interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg ADC_IT_EOC: End of conversion interrupt mask
  *     @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *     @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  * @retval The new state of ADC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetITStatus(mut ADCx: *mut ADC_TypeDef,
                                         mut ADC_IT: uint16_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itmask: uint32_t = 0 as libc::c_int as uint32_t;
    let mut enablestatus: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the ADC IT index */
    itmask = (ADC_IT as libc::c_int >> 8 as libc::c_int) as uint32_t;
    /* Get the ADC_IT enable bit status */
    enablestatus = (*ADCx).CR1 & ADC_IT as uint8_t as libc::c_uint;
    /* Check the status of the specified ADC interrupt */
    if (*ADCx).SR & itmask != RESET as libc::c_int as uint32_t &&
           enablestatus != 0 {
        /* ADC_IT is set */
        bitstatus = SET
    } else {
        /* ADC_IT is reset */
        bitstatus = RESET
    }
    /* Return the ADC_IT status */
    return bitstatus;
}
/* *
  * @}
  */
/* * @defgroup ADC_Exported_Constants
  * @{
  */
/* * @defgroup ADC_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_external_trigger_sources_for_regular_channels_conversion 
  * @{
  */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1, ADC2 and ADC3 */
/* !< For ADC1, ADC2 and ADC3 */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* *
  * @}
  */
/* * @defgroup ADC_data_align 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_channels 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_sampling_time 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_external_trigger_sources_for_injected_channels_conversion 
  * @{
  */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1 and ADC2 */
/* !< For ADC1, ADC2 and ADC3 */
/* !< For ADC1, ADC2 and ADC3 */
/* !< For ADC1, ADC2 and ADC3 */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* !< For ADC3 only */
/* *
  * @}
  */
/* * @defgroup ADC_injected_channel_selection 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_analog_watchdog_selection 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_thresholds 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_injected_offset 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_injected_length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_injected_rank 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_regular_length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_regular_rank 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_regular_discontinuous_mode_number 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup ADC_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup ADC_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the ADCx's interrupt pending bits.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_IT: specifies the ADC interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg ADC_IT_EOC: End of conversion interrupt mask
  *     @arg ADC_IT_AWD: Analog watchdog interrupt mask
  *     @arg ADC_IT_JEOC: End of injected conversion interrupt mask
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ClearITPendingBit(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_IT: uint16_t) {
    let mut itmask: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    /* Get the ADC IT index */
    itmask = (ADC_IT as libc::c_int >> 8 as libc::c_int) as uint8_t;
    /* Clear the selected ADC interrupt pending bits */
    ::core::ptr::write_volatile(&mut (*ADCx).SR as *mut uint32_t,
                                !(itmask as uint32_t));
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
