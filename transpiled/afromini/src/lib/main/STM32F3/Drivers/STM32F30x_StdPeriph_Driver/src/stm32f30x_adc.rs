use core;
use libc;
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
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ADC_TypeDef {
    pub ISR: uint32_t,
    pub IER: uint32_t,
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub RESERVED0: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub RESERVED1: uint32_t,
    pub TR1: uint32_t,
    pub TR2: uint32_t,
    pub TR3: uint32_t,
    pub RESERVED2: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub SQR4: uint32_t,
    pub DR: uint32_t,
    pub RESERVED3: uint32_t,
    pub RESERVED4: uint32_t,
    pub JSQR: uint32_t,
    pub RESERVED5: [uint32_t; 4],
    pub OFR1: uint32_t,
    pub OFR2: uint32_t,
    pub OFR3: uint32_t,
    pub OFR4: uint32_t,
    pub RESERVED6: [uint32_t; 4],
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub RESERVED7: [uint32_t; 4],
    pub AWD2CR: uint32_t,
    pub AWD3CR: uint32_t,
    pub RESERVED8: uint32_t,
    pub RESERVED9: uint32_t,
    pub DIFSEL: uint32_t,
    pub CALFACT: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ADC_Common_TypeDef {
    pub CSR: uint32_t,
    pub RESERVED: uint32_t,
    pub CCR: uint32_t,
    pub CDR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ADC_InitTypeDef {
    pub ADC_ContinuousConvMode: uint32_t,
    pub ADC_Resolution: uint32_t,
    pub ADC_ExternalTrigConvEvent: uint32_t,
    pub ADC_ExternalTrigEventEdge: uint32_t,
    pub ADC_DataAlign: uint32_t,
    pub ADC_OverrunMode: uint32_t,
    pub ADC_AutoInjMode: uint32_t,
    pub ADC_NbrOfRegChannel: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ADC_InjectedInitTypeDef {
    pub ADC_ExternalTrigInjecConvEvent: uint32_t,
    pub ADC_ExternalTrigInjecEventEdge: uint32_t,
    pub ADC_NbrOfInjecChannel: uint8_t,
    pub ADC_InjecSequence1: uint32_t,
    pub ADC_InjecSequence2: uint32_t,
    pub ADC_InjecSequence3: uint32_t,
    pub ADC_InjecSequence4: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct ADC_CommonInitTypeDef {
    pub ADC_Mode: uint32_t,
    pub ADC_Clock: uint32_t,
    pub ADC_DMAAccessMode: uint32_t,
    pub ADC_DMAMode: uint32_t,
    pub ADC_TwoSamplingDelay: uint8_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup ADC_Private_Functions
  * @{
  */
/* * @defgroup ADC_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
                 ##### Initialization and Configuration functions  #####
 ===============================================================================  
  [..] 
  This section provides functions allowing to:
   (#) Initialize and configure the ADC injected and/or regular channels and dual mode.
   (#) Management of the calibration process
   (#) ADC Power-on Power-off
   (#) Single ended or differential mode 
   (#) Enabling the queue of context and the auto delay mode
   (#) The number of ADC conversions that will be done using the sequencer for regular 
       channel group
   (#) Enable or disable the ADC peripheral
   
@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the ADCx peripheral registers to their default reset values.
  * @param  ADCx: where x can be 1, 2,3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DeInit(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        /* Enable ADC1/ADC2 reset state */
        RCC_AHBPeriphResetCmd(0x10000000i32 as uint32_t, ENABLE);
        /* Release ADC1/ADC2 from reset state */
        RCC_AHBPeriphResetCmd(0x10000000i32 as uint32_t, DISABLE);
    } else if ADCx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x10000000i32 as
                                                  libc::c_uint).wrapping_add(0x400i32
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut ADC_TypeDef ||
                  ADCx ==
                      (0x40000000i32 as
                           uint32_t).wrapping_add(0x10000000i32 as
                                                      libc::c_uint).wrapping_add(0x500i32
                                                                                     as
                                                                                     libc::c_uint)
                          as *mut ADC_TypeDef {
        /* Enable ADC3/ADC4 reset state */
        RCC_AHBPeriphResetCmd(0x20000000i32 as uint32_t, ENABLE);
        /* Release ADC3/ADC4 from reset state */
        RCC_AHBPeriphResetCmd(0x20000000i32 as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_InitStruct: pointer to an ADC_InitTypeDef structure that contains
  *         the configuration information for the specified ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_Init(mut ADCx: *mut ADC_TypeDef,
                                  mut ADC_InitStruct: *mut ADC_InitTypeDef) {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /*---------------------------- ADCx CFGR Configuration -----------------*/
  /* Get the ADCx CFGR value */
    tmpreg1 = (*ADCx).CFGR;
    /* Clear SCAN bit */
    tmpreg1 &= 0xfdffc007u32;
    /* Configure ADCx: scan conversion mode */
  /* Set SCAN bit according to ADC_ScanConvMode value */
    tmpreg1 |=
        (*ADC_InitStruct).ADC_ContinuousConvMode |
            (*ADC_InitStruct).ADC_Resolution |
            (*ADC_InitStruct).ADC_ExternalTrigConvEvent |
            (*ADC_InitStruct).ADC_ExternalTrigEventEdge |
            (*ADC_InitStruct).ADC_DataAlign |
            (*ADC_InitStruct).ADC_OverrunMode |
            (*ADC_InitStruct).ADC_AutoInjMode;
    /* Write to ADCx CFGR */
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t, tmpreg1);
    /*---------------------------- ADCx SQR1 Configuration -----------------*/
  /* Get the ADCx SQR1 value */
    tmpreg1 = (*ADCx).SQR1;
    /* Clear L bits */
    tmpreg1 &= !(0xfi32 as uint32_t);
    /* Configure ADCx: regular channel sequence length */
  /* Set L bits according to ADC_NbrOfRegChannel value */
    tmpreg1 |=
        ((*ADC_InitStruct).ADC_NbrOfRegChannel as libc::c_int - 1i32) as
            uint32_t;
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
    (*ADC_InitStruct).ADC_ContinuousConvMode =
        DISABLE as libc::c_int as uint32_t;
    (*ADC_InitStruct).ADC_Resolution = 0i32 as uint32_t;
    (*ADC_InitStruct).ADC_ExternalTrigConvEvent =
        0i32 as uint16_t as uint32_t;
    (*ADC_InitStruct).ADC_ExternalTrigEventEdge =
        0i32 as uint16_t as uint32_t;
    (*ADC_InitStruct).ADC_DataAlign = 0i32 as uint32_t;
    (*ADC_InitStruct).ADC_OverrunMode = DISABLE as libc::c_int as uint32_t;
    (*ADC_InitStruct).ADC_AutoInjMode = DISABLE as libc::c_int as uint32_t;
    (*ADC_InitStruct).ADC_NbrOfRegChannel = 1i32 as uint8_t;
}
/* *
  * @brief  Initializes the ADCx peripheral according to the specified parameters
  *         in the ADC_InitStruct.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_InjectInitStruct: pointer to an ADC_InjecInitTypeDef structure that contains
  *         the configuration information for the specified ADC injected channel.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_InjectedInit(mut ADCx: *mut ADC_TypeDef,
                                          mut ADC_InjectedInitStruct:
                                              *mut ADC_InjectedInitTypeDef) {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /*---------------------------- ADCx JSQR Configuration -----------------*/
  /* Get the ADCx JSQR value */
    tmpreg1 = (*ADCx).JSQR;
    /* Clear L bits */
    tmpreg1 &= 0i32 as uint32_t;
    /* Configure ADCx: Injected channel sequence length, external trigger, 
     external trigger edge and sequences
  */
    tmpreg1 =
        ((*ADC_InjectedInitStruct).ADC_NbrOfInjecChannel as libc::c_int -
             1i32 as uint8_t as libc::c_int) as libc::c_uint |
            (*ADC_InjectedInitStruct).ADC_ExternalTrigInjecConvEvent |
            (*ADC_InjectedInitStruct).ADC_ExternalTrigInjecEventEdge |
            (*ADC_InjectedInitStruct).ADC_InjecSequence1 << 8i32 |
            (*ADC_InjectedInitStruct).ADC_InjecSequence2 << 14i32 |
            (*ADC_InjectedInitStruct).ADC_InjecSequence3 << 20i32 |
            (*ADC_InjectedInitStruct).ADC_InjecSequence4 << 26i32;
    /* Write to ADCx SQR1 */
    ::core::ptr::write_volatile(&mut (*ADCx).JSQR as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Fills each ADC_InjectedInitStruct member with its default value.
  * @param  ADC_InjectedInitStruct : pointer to an ADC_InjectedInitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_InjectedStructInit(mut ADC_InjectedInitStruct:
                                                    *mut ADC_InjectedInitTypeDef) {
    (*ADC_InjectedInitStruct).ADC_ExternalTrigInjecConvEvent =
        0i32 as uint16_t as uint32_t;
    (*ADC_InjectedInitStruct).ADC_ExternalTrigInjecEventEdge =
        0i32 as uint16_t as uint32_t;
    (*ADC_InjectedInitStruct).ADC_NbrOfInjecChannel = 1i32 as uint8_t;
    (*ADC_InjectedInitStruct).ADC_InjecSequence1 =
        0x1i32 as uint8_t as uint32_t;
    (*ADC_InjectedInitStruct).ADC_InjecSequence2 =
        0x1i32 as uint8_t as uint32_t;
    (*ADC_InjectedInitStruct).ADC_InjecSequence3 =
        0x1i32 as uint8_t as uint32_t;
    (*ADC_InjectedInitStruct).ADC_InjecSequence4 =
        0x1i32 as uint8_t as uint32_t;
}
/* *
  * @brief  Initializes the ADCs peripherals according to the specified parameters 
  *         in the ADC_CommonInitStruct.
  * @param  ADCx: where x can be 1 or 4 to select the ADC peripheral.
  * @param  ADC_CommonInitStruct: pointer to an ADC_CommonInitTypeDef structure 
  *         that contains the configuration information for  All ADCs peripherals.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_CommonInit(mut ADCx: *mut ADC_TypeDef,
                                        mut ADC_CommonInitStruct:
                                            *mut ADC_CommonInitTypeDef) {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        /* Get the ADC CCR value */
        tmpreg1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        /* Clear MULTI, DELAY, DMA and ADCPRE bits */
        tmpreg1 &= 0xfffc10e0u32
    } else {
        /* Get the ADC CCR value */
        tmpreg1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x700i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        /* Clear MULTI, DELAY, DMA and ADCPRE bits */
        tmpreg1 &= 0xfffc10e0u32
    }
    /*---------------------------- ADC CCR Configuration -----------------*/  
  /* Configure ADCx: Multi mode, Delay between two sampling time, ADC clock, DMA mode
     and DMA access mode for dual mode */
  /* Set MULTI bits according to ADC_Mode value */
  /* Set CKMODE bits according to ADC_Clock value */
  /* Set MDMA bits according to ADC_DMAAccessMode value */
  /* Set DMACFG bits according to ADC_DMAMode value */
  /* Set DELAY bits according to ADC_TwoSamplingDelay value */
    tmpreg1 |=
        (*ADC_CommonInitStruct).ADC_Mode | (*ADC_CommonInitStruct).ADC_Clock |
            (*ADC_CommonInitStruct).ADC_DMAAccessMode |
            (*ADC_CommonInitStruct).ADC_DMAMode << 12i32 |
            ((*ADC_CommonInitStruct).ADC_TwoSamplingDelay as uint32_t) <<
                8i32;
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        /* Write to ADC CCR */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x10000000i32
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x300i32
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as
                                                *mut ADC_Common_TypeDef)).CCR
                                        as *mut uint32_t, tmpreg1)
    } else {
        /* Write to ADC CCR */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x10000000i32
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x700i32
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as
                                                *mut ADC_Common_TypeDef)).CCR
                                        as *mut uint32_t, tmpreg1)
    };
}
/* *
  * @brief  Fills each ADC_CommonInitStruct member with its default value.
  * @param  ADC_CommonInitStruct: pointer to an ADC_CommonInitTypeDef structure
  *         which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_CommonStructInit(mut ADC_CommonInitStruct:
                                                  *mut ADC_CommonInitTypeDef) {
    /* Initialize the ADC_Mode member */
    (*ADC_CommonInitStruct).ADC_Mode = 0i32 as uint32_t;
    /* initialize the ADC_Clock member */
    (*ADC_CommonInitStruct).ADC_Clock = 0i32 as uint32_t;
    /* Initialize the ADC_DMAAccessMode member */
    (*ADC_CommonInitStruct).ADC_DMAAccessMode = 0i32 as uint32_t;
    /* Initialize the ADC_DMAMode member */
    (*ADC_CommonInitStruct).ADC_DMAMode = 0i32 as uint32_t;
    /* Initialize the ADC_TwoSamplingDelay member */
    (*ADC_CommonInitStruct).ADC_TwoSamplingDelay = 0i32 as uint8_t;
}
/* *
  * @brief  Enables or disables the specified ADC peripheral.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx peripheral.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_Cmd(mut ADCx: *mut ADC_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the ADEN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x1i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC peripheral: Set the ADDIS bit */
        ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x2i32 as uint32_t)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Starts the selected ADC calibration process.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StartCalibration(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Set the ADCAL bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x80000000u32) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Returns the ADCx calibration value.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetCalibrationValue(mut ADCx: *mut ADC_TypeDef)
 -> uint32_t {
    /* Check the parameters */
    /* Return the selected ADC calibration value */
    return (*ADCx).CALFACT;
}
/* *
  * @brief  Sets the ADCx calibration register.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SetCalibrationValue(mut ADCx: *mut ADC_TypeDef,
                                                 mut ADC_Calibration:
                                                     uint32_t) {
    /* Check the parameters */
    /* Set the ADC calibration register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CALFACT as *mut uint32_t,
                                ADC_Calibration);
}
/* *
  * @brief  Select the ADC calibration mode.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_CalibrationMode: the ADC calibration mode.
  *         This parameter can be one of the following values: 
  *          @arg ADC_CalibrationMode_Single: to select the calibration for single channel
  *          @arg ADC_CalibrationMode_Differential: to select the calibration for differential channel         
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SelectCalibrationMode(mut ADCx: *mut ADC_TypeDef,
                                                   mut ADC_CalibrationMode:
                                                       uint32_t) {
    /* Check the parameters */
    /* Set or Reset the ADCALDIF bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x40000000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | ADC_CalibrationMode) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the selected ADC calibration status.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval The new state of ADC calibration (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetCalibrationStatus(mut ADCx: *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of CAL bit */
    if (*ADCx).CR & 0x80000000u32 != RESET as libc::c_int as uint32_t {
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
  * @brief  ADC Disable Command.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DisableCmd(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Set the ADDIS bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x2i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the selected ADC disable command Status.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval The new state of ADC ADC disable command (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetDisableCmdStatus(mut ADCx: *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of ADDIS bit */
    if (*ADCx).CR & 0x2i32 as uint32_t != RESET as libc::c_int as uint32_t {
        /* ADDIS bit is set */
        bitstatus = SET
    } else {
        /* ADDIS bit is reset */
        bitstatus = RESET
    }
    /* Return the ADDIS bit status */
    return bitstatus;
}
/* *
  * @brief  Enables or disables the specified ADC Voltage Regulator.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx Voltage Regulator.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_VoltageRegulatorCmd(mut ADCx: *mut ADC_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    /* set the intermediate state before moving the ADC voltage regulator 
  from enable state to disable state or from disable state to enable state */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x30000000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the ADVREGEN bit 0 */
        ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10000000i32 as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Set the ADVREGEN bit 1 */
        ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20000000i32 as uint32_t) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selectes the differential mode for a specific channel
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure for the analog watchdog. 
  *   This parameter can be one of the following values:
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
  * @note : Channel 15, 16 and 17 are fixed to single-ended inputs mode.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SelectDifferentialMode(mut ADCx:
                                                        *mut ADC_TypeDef,
                                                    mut ADC_Channel: uint8_t,
                                                    mut NewState:
                                                        FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the DIFSEL bit */
        ::core::ptr::write_volatile(&mut (*ADCx).DIFSEL as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).DIFSEL
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (1i32 << ADC_Channel as libc::c_int)
                                             as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Reset the DIFSEL bit */
        ::core::ptr::write_volatile(&mut (*ADCx).DIFSEL as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).DIFSEL
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((1i32 <<
                                                ADC_Channel as libc::c_int) as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Selects the Queue Of Context Mode for injected channels.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the Queue Of Context Mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SelectQueueOfContextMode(mut ADCx:
                                                          *mut ADC_TypeDef,
                                                      mut NewState:
                                                          FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the JQM bit */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x200000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Reset the JQM bit */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x200000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the ADC Delayed Conversion Mode.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADC Delayed Conversion Mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AutoDelayCmd(mut ADCx: *mut ADC_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the AUTDLY bit */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4000i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Reset the AUTDLY bit */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4000i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup ADC_Group2 Analog Watchdog configuration functions
 *  @brief   Analog Watchdog configuration functions 
 *
@verbatim   
 ===============================================================================
                    ##### Analog Watchdog configuration functions #####
 ===============================================================================  

  [..] This section provides functions allowing to configure the 3 Analog Watchdogs 
       (AWDG1, AWDG2 and AWDG3) in the ADC.
  
  [..] A typical configuration Analog Watchdog is done following these steps :
   (#) The ADC guarded channel(s) is (are) selected using the functions: 
      (++) ADC_AnalogWatchdog1SingleChannelConfig().
      (++) ADC_AnalogWatchdog2SingleChannelConfig().
      (++) ADC_AnalogWatchdog3SingleChannelConfig().

   (#) The Analog watchdog lower and higher threshold are configured using the functions: 
      (++) ADC_AnalogWatchdog1ThresholdsConfig().
      (++) ADC_AnalogWatchdog2ThresholdsConfig().
      (++) ADC_AnalogWatchdog3ThresholdsConfig().

   (#) The Analog watchdog is enabled and configured to enable the check, on one
      or more channels, using the function:
      (++) ADC_AnalogWatchdogCmd().

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the analog watchdog on single/all regular
  *         or injected channels
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
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
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).CFGR;
    /* Clear AWDEN, AWDENJ and AWDSGL bits */
    tmpreg &=
        !(0x400000i32 as uint32_t | 0x800000i32 as uint32_t |
              0x1000000i32 as uint32_t);
    /* Set the analog watchdog enable mode */
    tmpreg |= ADC_AnalogWatchdog;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the high and low thresholds of the analog watchdog1.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  HighThreshold: the ADC analog watchdog High threshold value.
  *   This parameter must be a 12bit value.
  * @param  LowThreshold: the ADC analog watchdog Low threshold value.
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdog1ThresholdsConfig(mut ADCx:
                                                                 *mut ADC_TypeDef,
                                                             mut HighThreshold:
                                                                 uint16_t,
                                                             mut LowThreshold:
                                                                 uint16_t) {
    /* Check the parameters */
    /* Set the ADCx high threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).TR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xfff0000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).TR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (HighThreshold as uint32_t) << 16i32) as
                                    uint32_t as uint32_t);
    /* Set the ADCx low threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).TR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xfffi32 as uint32_t)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).TR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     LowThreshold as libc::c_uint) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Configures the high and low thresholds of the analog watchdog2.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  HighThreshold: the ADC analog watchdog High threshold value.
  *   This parameter must be a 8bit value.
  * @param  LowThreshold: the ADC analog watchdog Low threshold value.
  *   This parameter must be a 8bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdog2ThresholdsConfig(mut ADCx:
                                                                 *mut ADC_TypeDef,
                                                             mut HighThreshold:
                                                                 uint8_t,
                                                             mut LowThreshold:
                                                                 uint8_t) {
    /* Check the parameters */
    /* Set the ADCx high threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).TR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xff0000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).TR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (HighThreshold as uint32_t) << 16i32) as
                                    uint32_t as uint32_t);
    /* Set the ADCx low threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).TR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0xffi32 as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).TR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     LowThreshold as libc::c_uint) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Configures the high and low thresholds of the analog watchdog3.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  HighThreshold: the ADC analog watchdog High threshold value.
  *   This parameter must be a 8bit value.
  * @param  LowThreshold: the ADC analog watchdog Low threshold value.
  *   This parameter must be a 8bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdog3ThresholdsConfig(mut ADCx:
                                                                 *mut ADC_TypeDef,
                                                             mut HighThreshold:
                                                                 uint8_t,
                                                             mut LowThreshold:
                                                                 uint8_t) {
    /* Check the parameters */
    /* Set the ADCx high threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).TR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xff0000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).TR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (HighThreshold as uint32_t) << 16i32) as
                                    uint32_t as uint32_t);
    /* Set the ADCx low threshold */
    ::core::ptr::write_volatile(&mut (*ADCx).TR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0xffi32 as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).TR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).TR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     LowThreshold as libc::c_uint) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Configures the analog watchdog 2 guarded single channel
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure for the analog watchdog. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdog1SingleChannelConfig(mut ADCx:
                                                                    *mut ADC_TypeDef,
                                                                mut ADC_Channel:
                                                                    uint8_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).CFGR;
    /* Clear the Analog watchdog channel select bits */
    tmpreg &= !(0x7c000000i32 as uint32_t);
    /* Set the Analog watchdog channel */
    tmpreg |= (ADC_Channel as uint32_t) << 26i32;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the analog watchdog 2 guarded single channel
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure for the analog watchdog. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdog2SingleChannelConfig(mut ADCx:
                                                                    *mut ADC_TypeDef,
                                                                mut ADC_Channel:
                                                                    uint8_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).AWD2CR;
    /* Clear the Analog watchdog channel select bits */
    tmpreg &= !(0x7fffei32 as uint32_t);
    /* Set the Analog watchdog channel */
    tmpreg |= (1i32 as uint32_t) << ADC_Channel as libc::c_int;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).AWD2CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).AWD2CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | tmpreg) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Configures the analog watchdog 3 guarded single channel
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure for the analog watchdog. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_AnalogWatchdog3SingleChannelConfig(mut ADCx:
                                                                    *mut ADC_TypeDef,
                                                                mut ADC_Channel:
                                                                    uint8_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*ADCx).AWD3CR;
    /* Clear the Analog watchdog channel select bits */
    tmpreg &= !(0x7fffei32 as uint32_t);
    /* Set the Analog watchdog channel */
    tmpreg |= (1i32 as uint32_t) << ADC_Channel as libc::c_int;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).AWD3CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).AWD3CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | tmpreg) as uint32_t as
                                    uint32_t);
}
/* *
  * @}
  */
/* * @defgroup ADC_Group3 Temperature Sensor - Vrefint (Internal Reference Voltage) and VBAT management functions
 *  @brief   Vbat, Temperature Sensor & Vrefint (Internal Reference Voltage) management function 
 *
@verbatim   
 ====================================================================================================
  ##### Temperature Sensor - Vrefint (Internal Reference Voltage) and VBAT management functions #####
 ====================================================================================================  

  [..] This section provides a function allowing to enable/ disable the internal 
  connections between the ADC and the Vbat/2, Temperature Sensor and the Vrefint source.

  [..] A typical configuration to get the Temperature sensor and Vrefint channels 
  voltages is done following these steps :
   (#) Enable the internal connection of Vbat/2, Temperature sensor and Vrefint sources 
       with the ADC channels using:
      (++) ADC_TempSensorCmd()  
      (++) ADC_VrefintCmd() 
      (++) ADC_VbatCmd()  

   (#) select the ADC_Channel_TempSensor and/or ADC_Channel_Vrefint and/or ADC_Channel_Vbat using 
      (++) ADC_RegularChannelConfig() or  
      (++) ADC_InjectedInit() functions 

   (#) Get the voltage values, using:
      (++) ADC_GetConversionValue() or  
      (++) ADC_GetInjectedConversionValue().
 
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the temperature sensor channel.
  * @param  ADCx: where x can be 1 to select the ADC peripheral.
  * @param  NewState: new state of the temperature sensor.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_TempSensorCmd(mut ADCx: *mut ADC_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the temperature sensor channel*/
        let ref mut fresh0 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the temperature sensor channel*/
        let ref mut fresh1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Vrefint channel.
  * @param  ADCx: where x can be 1 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the Vrefint.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_VrefintCmd(mut ADCx: *mut ADC_TypeDef,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint
           {
            /* Enable the Vrefint channel*/
            let ref mut fresh2 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x10000000i32 as
                                                   libc::c_uint).wrapping_add(0x300i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut ADC_Common_TypeDef)).CCR;
            ::core::ptr::write_volatile(fresh2,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x400000i32 as uint32_t) as
                                            uint32_t as uint32_t)
        } else {
            /* Disable the Vrefint channel*/
            let ref mut fresh3 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x10000000i32 as
                                                   libc::c_uint).wrapping_add(0x300i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut ADC_Common_TypeDef)).CCR;
            ::core::ptr::write_volatile(fresh3,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x400000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    } else if NewState as libc::c_uint !=
                  DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Vrefint channel*/
        let ref mut fresh4 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x700i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x400000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the Vrefint channel*/
        let ref mut fresh5 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x700i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x400000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Vbat channel.
  * @param  ADCx: where x can be 1 to select the ADC peripheral.
  * @param  NewState: new state of the Vbat.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_VbatCmd(mut ADCx: *mut ADC_TypeDef,
                                     mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Vbat channel*/
        let ref mut fresh6 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the Vbat channel*/
        let ref mut fresh7 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CCR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup ADC_Group4 Regular Channels Configuration functions
 *  @brief   Regular Channels Configuration functions 
 *
@verbatim   
 ===============================================================================
                  ##### Channels Configuration functions #####
 ===============================================================================  

  [..] This section provides functions allowing to manage the ADC regular channels.
   
  [..] To configure a regular sequence of channels use:
   (#) ADC_RegularChannelConfig()
       this fuction allows:
       (++) Configure the rank in the regular group sequencer for each channel
       (++) Configure the sampling time for each channel

   (#) ADC_RegularChannelSequencerLengthConfig() to set the length of the regular sequencer

   [..] The regular trigger is configured using the following functions:
   (#) ADC_SelectExternalTrigger()
   (#) ADC_ExternalTriggerPolarityConfig()

   [..] The start and the stop conversion are controlled by:
   (#) ADC_StartConversion()
   (#) ADC_StopConversion()
    
   [..] 
   (@)Please Note that the following features for regular channels are configurated
     using the ADC_Init() function : 
          (++) continuous mode activation
          (++) Resolution  
          (++) Data Alignement 
          (++) Overrun Mode.
     
  [..] Get the conversion data: This subsection provides an important function in 
     the ADC peripheral since it returns the converted data of the current 
     regular channel. When the Conversion value is read, the EOC Flag is 
     automatically cleared.

  [..] To configure the  discontinous mode, the following functions should be used:
   (#) ADC_DiscModeChannelCountConfig() to configure the number of discontinuous channel to be converted.
   (#) ADC_DiscModeCmd() to enable the discontinuous mode.

  [..] To configure and enable/disable the Channel offset use the functions:
     (++) ADC_SetChannelOffset1()
     (++) ADC_SetChannelOffset2()
     (++) ADC_SetChannelOffset3()
     (++) ADC_SetChannelOffset4()
     (++) ADC_ChannelOffset1Cmd()
     (++) ADC_ChannelOffset2Cmd()
     (++) ADC_ChannelOffset3Cmd()
     (++) ADC_ChannelOffset4Cmd()
  
@endverbatim
  * @{
  */
/* *
  * @brief  Configures for the selected ADC regular channel its corresponding
  *         rank in the sequencer and its sample time.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @param  Rank: The rank in the regular group sequencer. This parameter must be between 1 to 16.
  * @param  ADC_SampleTime: The sample time value to be set for the selected channel. 
  *   This parameter can be one of the following values:
  *     @arg ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
  *     @arg ADC_SampleTime_2Cycles5: Sample time equal to 2.5 cycles
  *     @arg ADC_SampleTime_4Cycles5: Sample time equal to 4.5 cycles
  *     @arg ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles	
  *     @arg ADC_SampleTime_19Cycles5: Sample time equal to 19.5 cycles	
  *     @arg ADC_SampleTime_61Cycles5: Sample time equal to 61.5 cycles	
  *     @arg ADC_SampleTime_181Cycles5: Sample time equal to 181.5 cycles	
  *     @arg ADC_SampleTime_601Cycles5: Sample time equal to 601.5 cycles	
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_RegularChannelConfig(mut ADCx: *mut ADC_TypeDef,
                                                  mut ADC_Channel: uint8_t,
                                                  mut Rank: uint8_t,
                                                  mut ADC_SampleTime:
                                                      uint8_t) {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    let mut tmpreg2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Regular sequence configuration */
  /* For Rank 1 to 4 */
    if (Rank as libc::c_int) < 5i32 {
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR1;
        /* Calculate the mask to clear */
        tmpreg2 = (0x1fi32 << 6i32 * Rank as libc::c_int) as uint32_t;
        /* Clear the old SQx bits for the selected rank */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 = (ADC_Channel as uint32_t) << 6i32 * Rank as libc::c_int;
        /* Set the SQx bits for the selected rank */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SQR1 as *mut uint32_t,
                                    tmpreg1)
    } else if (Rank as libc::c_int) < 10i32 {
        /* For Rank 5 to 9 */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR2;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x1fi32 as uint32_t) << 6i32 * (Rank as libc::c_int - 5i32);
        /* Clear the old SQx bits for the selected rank */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_Channel as uint32_t) << 6i32 * (Rank as libc::c_int - 5i32);
        /* Set the SQx bits for the selected rank */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SQR2 as *mut uint32_t,
                                    tmpreg1)
    } else if (Rank as libc::c_int) < 15i32 {
        /* For Rank 10 to 14 */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR3;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x1fi32 as uint32_t) << 6i32 * (Rank as libc::c_int - 10i32);
        /* Clear the old SQx bits for the selected rank */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_Channel as uint32_t) << 6i32 * (Rank as libc::c_int - 10i32);
        /* Set the SQx bits for the selected rank */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SQR3 as *mut uint32_t,
                                    tmpreg1)
    } else {
        /* Get the old register value */
        tmpreg1 = (*ADCx).SQR4;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x1fi32 as uint32_t) << 6i32 * (Rank as libc::c_int - 15i32);
        /* Clear the old SQx bits for the selected rank */
        tmpreg1 &= !tmpreg2;
        /* Calculate the mask to set */
        tmpreg2 =
            (ADC_Channel as uint32_t) << 6i32 * (Rank as libc::c_int - 15i32);
        /* Set the SQx bits for the selected rank */
        tmpreg1 |= tmpreg2;
        /* Store the new register value */
        ::core::ptr::write_volatile(&mut (*ADCx).SQR4 as *mut uint32_t,
                                    tmpreg1)
    }
    /* Channel sampling configuration */
  /* if ADC_Channel_10 ... ADC_Channel_18 is selected */
    if ADC_Channel as libc::c_int > 0x9i32 as uint8_t as libc::c_int {
        /* Get the old register value */
        tmpreg1 = (*ADCx).SMPR2;
        /* Calculate the mask to clear */
        tmpreg2 =
            (0x7i32 as uint32_t) <<
                3i32 * (ADC_Channel as libc::c_int - 10i32);
        /* Clear the old channel sample time */
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !tmpreg2) as
                                        uint32_t as uint32_t);
        /* Calculate the mask to set */
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (ADC_SampleTime as uint32_t) <<
                                             3i32 *
                                                 (ADC_Channel as libc::c_int -
                                                      10i32)) as uint32_t as
                                        uint32_t)
    } else {
        /* ADC_Channel include in ADC_Channel_[0..9] */
        /* Get the old register value */
        tmpreg1 = (*ADCx).SMPR1;
        tmpreg2 =
            (0x38i32 as uint32_t) <<
                3i32 * (ADC_Channel as libc::c_int - 1i32);
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !tmpreg2) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (ADC_SampleTime as uint32_t) <<
                                             3i32 *
                                                 ADC_Channel as libc::c_int)
                                        as uint32_t as uint32_t)
    };
}
/* Calculate the mask to clear */
/* Clear the old channel sample time */
/* Calculate the mask to set */
/* *
  * @brief  Sets the ADC regular channel sequence lenght.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  SequenceLength: The Regular sequence length. This parameter must be between 1 to 16.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_RegularChannelSequencerLengthConfig(mut ADCx:
                                                                     *mut ADC_TypeDef,
                                                                 mut SequencerLength:
                                                                     uint8_t) {
    /* Check the parameters */
    /* Configure the ADC sequence lenght */
    ::core::ptr::write_volatile(&mut (*ADCx).SQR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SQR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0xfi32 as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).SQR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SQR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (SequencerLength as libc::c_int - 1i32)
                                         as uint32_t) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  External Trigger Enable and Polarity Selection for regular channels.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_ExternalTrigConvEvent: ADC external Trigger source.
  *   This parameter can be one of the following values:
  *     @arg ADC_ExternalTrigger_Event0: External trigger event 0 
  *     @arg ADC_ExternalTrigger_Event1: External trigger event 1
  *     @arg ADC_ExternalTrigger_Event2: External trigger event 2
  *     @arg ADC_ExternalTrigger_Event3: External trigger event 3
  *     @arg ADC_ExternalTrigger_Event4: External trigger event 4 
  *     @arg ADC_ExternalTrigger_Event5: External trigger event 5
  *     @arg ADC_ExternalTrigger_Event6: External trigger event 6
  *     @arg ADC_ExternalTrigger_Event7: External trigger event 7
  *     @arg ADC_ExternalTrigger_Event8: External trigger event 8 
  *     @arg ADC_ExternalTrigger_Event9: External trigger event 9
  *     @arg ADC_ExternalTrigger_Event10: External trigger event 10
  *     @arg ADC_ExternalTrigger_Event11: External trigger event 11
  *     @arg ADC_ExternalTrigger_Event12: External trigger event 12 
  *     @arg ADC_ExternalTrigger_Event13: External trigger event 13
  *     @arg ADC_ExternalTrigger_Event14: External trigger event 14
  *     @arg ADC_ExternalTrigger_Event15: External trigger event 15	  
  * @param  ADC_ExternalTrigEventEdge: ADC external Trigger Polarity.
  *   This parameter can be one of the following values:
  *     @arg ADC_ExternalTrigEventEdge_OFF: Hardware trigger detection disabled 
  *                                          (conversions can be launched by software)
  *     @arg ADC_ExternalTrigEventEdge_RisingEdge: Hardware trigger detection on the rising edge
  *     @arg ADC_ExternalTrigEventEdge_FallingEdge: Hardware trigger detection on the falling edge
  *     @arg ADC_ExternalTrigEventEdge_BothEdge: Hardware trigger detection on both the rising and falling edges	
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ExternalTriggerConfig(mut ADCx: *mut ADC_TypeDef,
                                                   mut ADC_ExternalTrigConvEvent:
                                                       uint16_t,
                                                   mut ADC_ExternalTrigEventEdge:
                                                       uint16_t) {
    /* Check the parameters */
    /* Disable the selected ADC conversion on external event */
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xc00i32 as uint32_t |
                                           0x3c0i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (ADC_ExternalTrigEventEdge as libc::c_int
                                          |
                                          ADC_ExternalTrigConvEvent as
                                              libc::c_int) as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the selected ADC start conversion .
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StartConversion(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Set the ADSTART bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x4i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the selected ADC start conversion Status.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval The new state of ADC start conversion (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetStartConversionStatus(mut ADCx:
                                                          *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of ADSTART bit */
    if (*ADCx).CR & 0x4i32 as uint32_t != RESET as libc::c_int as uint32_t {
        /* ADSTART bit is set */
        bitstatus = SET
    } else {
        /* ADSTART bit is reset */
        bitstatus = RESET
    }
    /* Return the ADSTART bit status */
    return bitstatus;
}
/* *
  * @brief  Stops the selected ADC ongoing conversion.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StopConversion(mut ADCx: *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Set the ADSTP bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x10i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the discontinuous mode for the selected ADC regular
  *         group channel.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  Number: specifies the discontinuous mode regular channel
  *         count value. This number must be between 1 and 8.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DiscModeChannelCountConfig(mut ADCx:
                                                            *mut ADC_TypeDef,
                                                        mut Number: uint8_t) {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    let mut tmpreg2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg1 = (*ADCx).CFGR;
    /* Clear the old discontinuous mode channel count */
    tmpreg1 &= !(0xe0000i32 as uint32_t);
    /* Set the discontinuous mode channel count */
    tmpreg2 = (Number as libc::c_int - 1i32) as uint32_t;
    tmpreg1 |= tmpreg2 << 17i32;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Enables or disables the discontinuous mode on regular group
  *         channel for the specified ADC
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
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
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC regular discontinuous mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Returns the last ADCx conversion result data for regular channel.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
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
  * @brief  Returns the last ADC1, ADC2, ADC3 and ADC4 regular conversions results 
  *         data in the selected dual mode.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.  
  * @retval The Data conversion value.
  * @note   In dual mode, the value returned by this function is as following
  *           Data[15:0] : these bits contain the regular data of the Master ADC.
  *           Data[31:16]: these bits contain the regular data of the Slave ADC.           
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetDualModeConversionValue(mut ADCx:
                                                            *mut ADC_TypeDef)
 -> uint32_t {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        /* Get the dual mode conversion value */
        tmpreg1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CDR
    } else {
        /* Get the dual mode conversion value */
        tmpreg1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x700i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CDR
    }
    /* Return the dual mode conversion value */
    return tmpreg1;
}
/* *
  * @brief  Set the ADC channels conversion value offset1
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @param  Offset: the offset value for the selected ADC Channel
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SetChannelOffset1(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_Channel: uint8_t,
                                               mut Offset: uint16_t) {
    /* Check the parameters */
    /* Select the Channel */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x7c000000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (ADC_Channel as uint32_t) << 26i32) as
                                    uint32_t as uint32_t);
    /* Set the data offset */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xfffi32 as uint32_t)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Offset as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Set the ADC channels conversion value offset2
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @param  Offset: the offset value for the selected ADC Channel
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SetChannelOffset2(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_Channel: uint8_t,
                                               mut Offset: uint16_t) {
    /* Check the parameters */
    /* Select the Channel */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x7c000000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (ADC_Channel as uint32_t) << 26i32) as
                                    uint32_t as uint32_t);
    /* Set the data offset */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xfffi32 as uint32_t)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Offset as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Set the ADC channels conversion value offset3
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @param  Offset: the offset value for the selected ADC Channel
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SetChannelOffset3(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_Channel: uint8_t,
                                               mut Offset: uint16_t) {
    /* Check the parameters */
    /* Select the Channel */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x7c000000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (ADC_Channel as uint32_t) << 26i32) as
                                    uint32_t as uint32_t);
    /* Set the data offset */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xfffi32 as uint32_t)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR3 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Offset as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Set the ADC channels conversion value offset4
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
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
  *     @arg ADC_Channel_18: ADC Channel18 selected
  * @param  Offset: the offset value for the selected ADC Channel
  *   This parameter must be a 12bit value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_SetChannelOffset4(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_Channel: uint8_t,
                                               mut Offset: uint16_t) {
    /* Check the parameters */
    /* Select the Channel */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR4 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x7c000000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR4 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (ADC_Channel as uint32_t) << 26i32) as
                                    uint32_t as uint32_t);
    /* Set the data offset */
    ::core::ptr::write_volatile(&mut (*ADCx).OFR4 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xfffi32 as uint32_t)) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).OFR4 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Offset as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the Offset1.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx offset1.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ChannelOffset1Cmd(mut ADCx: *mut ADC_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x80000000u32) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !0x80000000u32) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Offset2.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx offset2.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ChannelOffset2Cmd(mut ADCx: *mut ADC_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x80000000u32) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !0x80000000u32) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Offset3.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx offset3.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ChannelOffset3Cmd(mut ADCx: *mut ADC_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x80000000u32) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR3 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !0x80000000u32) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Offset4.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the ADCx offset4.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ChannelOffset4Cmd(mut ADCx: *mut ADC_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR4 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x80000000u32) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the OFFSET1_EN bit */
        ::core::ptr::write_volatile(&mut (*ADCx).OFR4 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).OFR4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !0x80000000u32) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup ADC_Group5 Regular Channels DMA Configuration functions
 *  @brief   Regular Channels DMA Configuration functions 
 *
@verbatim   
 ===============================================================================
                   ##### Regular Channels DMA Configuration functions #####
 ===============================================================================  

  [..] This section provides functions allowing to configure the DMA for ADC regular 
  channels. Since converted regular channel values are stored into a unique data register, 
  it is useful to use DMA for conversion of more than one regular channel. This 
  avoids the loss of the data already stored in the ADC Data register. 
  
  (#) ADC_DMACmd() function is used to enable the ADC DMA mode, after each
      conversion of a regular channel, a DMA request is generated.
  (#) ADC_DMAConfig() function is used to select between the one shot DMA mode 
      or the circular DMA mode

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified ADC DMA request.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
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
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x1i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the selected ADC DMA request */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1i32 as uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Configure ADC DMA mode.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_DMAMode: select the ADC DMA mode.
  *   This parameter can be one of the following values:
  *     @arg ADC_DMAMode_OneShot: ADC DMA Oneshot mode
  *     @arg ADC_DMAMode_Circular: ADC DMA circular mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_DMAConfig(mut ADCx: *mut ADC_TypeDef,
                                       mut ADC_DMAMode: uint32_t) {
    /* Check the parameters */
    /* Set or reset the DMACFG bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x2i32 as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | ADC_DMAMode) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup ADC_Group6 Injected channels Configuration functions
 *  @brief   Injected channels Configuration functions 
 *
@verbatim   
 ===============================================================================
                     ##### Injected channels Configuration functions #####
 ===============================================================================  

  [..] This section provide functions allowing to manage the ADC Injected channels,
  it is composed of : 
    
   (#) Configuration functions for Injected channels sample time
   (#) Functions to start and stop the injected conversion
   (#) unction to select the discontinuous mode    
   (#) Function to get the Specified Injected channel conversion data: This subsection 
      provides an important function in the ADC peripheral since it returns the 
      converted data of the specific injected channel.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures for the selected ADC injected channel its corresponding
  *         sample time.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_Channel: the ADC channel to configure. 
  *   This parameter can be one of the following values:
  *     @arg ADC_InjectedChannel_1: ADC Channel1 selected
  *     @arg ADC_InjectedChannel_2: ADC Channel2 selected
  *     @arg ADC_InjectedChannel_3: ADC Channel3 selected
  *     @arg ADC_InjectedChannel_4: ADC Channel4 selected
  *     @arg ADC_InjectedChannel_5: ADC Channel5 selected
  *     @arg ADC_InjectedChannel_6: ADC Channel6 selected
  *     @arg ADC_InjectedChannel_7: ADC Channel7 selected
  *     @arg ADC_InjectedChannel_8: ADC Channel8 selected
  *     @arg ADC_InjectedChannel_9: ADC Channel9 selected
  *     @arg ADC_InjectedChannel_10: ADC Channel10 selected
  *     @arg ADC_InjectedChannel_11: ADC Channel11 selected
  *     @arg ADC_InjectedChannel_12: ADC Channel12 selected
  *     @arg ADC_InjectedChannel_13: ADC Channel13 selected
  *     @arg ADC_InjectedChannel_14: ADC Channel14 selected
  *     @arg ADC_InjectedChannel_15: ADC Channel15 selected
  *     @arg ADC_InjectedChannel_16: ADC Channel16 selected
  *     @arg ADC_InjectedChannel_17: ADC Channel17 selected
  *     @arg ADC_InjectedChannel_18: ADC Channel18 selected
  * @param  ADC_SampleTime: The sample time value to be set for the selected channel. 
  *   This parameter can be one of the following values:
  *     @arg ADC_SampleTime_1Cycles5: Sample time equal to 1.5 cycles
  *     @arg ADC_SampleTime_2Cycles5: Sample time equal to 2.5 cycles
  *     @arg ADC_SampleTime_4Cycles5: Sample time equal to 4.5 cycles
  *     @arg ADC_SampleTime_7Cycles5: Sample time equal to 7.5 cycles	
  *     @arg ADC_SampleTime_19Cycles5: Sample time equal to 19.5 cycles	
  *     @arg ADC_SampleTime_61Cycles5: Sample time equal to 61.5 cycles	
  *     @arg ADC_SampleTime_181Cycles5: Sample time equal to 181.5 cycles	
  *     @arg ADC_SampleTime_601Cycles5: Sample time equal to 601.5 cycles	
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_InjectedChannelSampleTimeConfig(mut ADCx:
                                                                 *mut ADC_TypeDef,
                                                             mut ADC_InjectedChannel:
                                                                 uint8_t,
                                                             mut ADC_SampleTime:
                                                                 uint8_t) {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Channel sampling configuration */
  /* if ADC_InjectedChannel_10 ... ADC_InjectedChannel_18 is selected */
    if ADC_InjectedChannel as libc::c_int > 0x9i32 as uint8_t as libc::c_int {
        /* Calculate the mask to clear */
        tmpreg1 =
            (0x7i32 as uint32_t) <<
                3i32 * (ADC_InjectedChannel as libc::c_int - 10i32);
        /* Clear the old channel sample time */
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !tmpreg1) as
                                        uint32_t as uint32_t);
        /* Calculate the mask to set */
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (ADC_SampleTime as uint32_t) <<
                                             3i32 *
                                                 (ADC_InjectedChannel as
                                                      libc::c_int - 10i32)) as
                                        uint32_t as uint32_t)
    } else {
        /* ADC_InjectedChannel include in ADC_InjectedChannel_[0..9] */
        /* Calculate the mask to clear */
        tmpreg1 =
            (0x38i32 as uint32_t) <<
                3i32 * (ADC_InjectedChannel as libc::c_int - 1i32);
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !tmpreg1) as
                                        uint32_t as uint32_t);
        ::core::ptr::write_volatile(&mut (*ADCx).SMPR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).SMPR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (ADC_SampleTime as uint32_t) <<
                                             3i32 *
                                                 ADC_InjectedChannel as
                                                     libc::c_int) as uint32_t
                                        as uint32_t)
    };
}
/* Clear the old channel sample time */
/* Calculate the mask to set */
/* *
  * @brief  Enables or disables the selected ADC start of the injected 
  *         channels conversion.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  NewState: new state of the selected ADC software start injected conversion.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StartInjectedConversion(mut ADCx:
                                                         *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Enable the selected ADC conversion for injected group on external event and start the selected
     ADC injected conversion */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x8i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Stops the selected ADC ongoing injected conversion.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_StopInjectedConversion(mut ADCx:
                                                        *mut ADC_TypeDef) {
    /* Check the parameters */
    /* Set the JADSTP bit */
    ::core::ptr::write_volatile(&mut (*ADCx).CR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x20i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Gets the selected ADC Software start injected conversion Status.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @retval The new state of ADC start injected conversion (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetStartInjectedConversionStatus(mut ADCx:
                                                                  *mut ADC_TypeDef)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of JADSTART bit */
    if (*ADCx).CR & 0x8i32 as uint32_t != RESET as libc::c_int as uint32_t {
        /* JADSTART bit is set */
        bitstatus = SET
    } else {
        /* JADSTART bit is reset */
        bitstatus = RESET
    }
    /* Return the JADSTART bit status */
    return bitstatus;
}
/* *
  * @brief  Enables or disables the selected ADC automatic injected group
  *         conversion after regular one.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
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
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2000000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC automatic injected group conversion */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2000000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the discontinuous mode for injected group
  *         channel for the specified ADC
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
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
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x100000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC injected discontinuous mode */
        ::core::ptr::write_volatile(&mut (*ADCx).CFGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).CFGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x100000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Returns the ADC injected channel conversion result
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_InjectedSequence: the converted ADC injected sequence.
  *   This parameter can be one of the following values:
  *     @arg ADC_InjectedSequence_1: Injected Sequence1 selected
  *     @arg ADC_InjectedSequence_2: Injected Sequence2 selected
  *     @arg ADC_InjectedSequence_3: Injected Sequence3 selected
  *     @arg ADC_InjectedSequence_4: Injected Sequence4 selected
  * @retval The Data conversion value.
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetInjectedConversionValue(mut ADCx:
                                                            *mut ADC_TypeDef,
                                                        mut ADC_InjectedSequence:
                                                            uint8_t)
 -> uint16_t {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, ADCx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(((ADC_InjectedSequence
                                                                      as
                                                                      libc::c_int
                                                                      - 1i32
                                                                      << 2i32)
                                                                     +
                                                                     0x80i32
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
  * @}
  */
/* * @defgroup ADC_Group7 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
 ===============================================================================
                   ##### Interrupts and flags management functions #####
 ===============================================================================  

  [..] This section provides functions allowing to configure the ADC Interrupts, get 
        the status and clear flags and Interrupts pending bits.
  
  [..] The ADC provide 11 Interrupts sources and 11 Flags which can be divided into 3 groups:
  
  (#) Flags and Interrupts for ADC regular channels
  (##)Flags
      (+) ADC_FLAG_RDY: ADC Ready flag
      (+) ADC_FLAG_EOSMP: ADC End of Sampling flag
      (+) ADC_FLAG_EOC: ADC End of Regular Conversion flag.
      (+) ADC_FLAG_EOS: ADC End of Regular sequence of Conversions flag
      (+) ADC_FLAG_OVR: ADC overrun flag
     
  (##) Interrupts
      (+) ADC_IT_RDY: ADC Ready interrupt source 
      (+) ADC_IT_EOSMP: ADC End of Sampling interrupt source
      (+) ADC_IT_EOC: ADC End of Regular Conversion interrupt source
      (+) ADC_IT_EOS: ADC End of Regular sequence of Conversions interrupt
      (+) ADC_IT_OVR: ADC overrun interrupt source
  
  
  (#) Flags and Interrupts for ADC regular channels
  (##)Flags
      (+) ADC_FLAG_JEOC: ADC Ready flag
      (+) ADC_FLAG_JEOS: ADC End of Sampling flag
      (+) ADC_FLAG_JQOVF: ADC End of Regular Conversion flag.
     
  (##) Interrupts
      (+) ADC_IT_JEOC: ADC End of Injected Conversion interrupt source 
      (+) ADC_IT_JEOS: ADC End of Injected sequence of Conversions interrupt source
      (+) ADC_IT_JQOVF: ADC Injected Context Queue Overflow interrupt source   

  (#) General Flags and Interrupts for the ADC
  (##)Flags 
     (+)  ADC_FLAG_AWD1: ADC Analog watchdog 1 flag
     (+) ADC_FLAG_AWD2: ADC Analog watchdog 2 flag
     (+) ADC_FLAG_AWD3: ADC Analog watchdog 3 flag
    
  (##)Flags 
     (+)  ADC_IT_AWD1: ADC Analog watchdog 1 interrupt source
     (+) ADC_IT_AWD2: ADC Analog watchdog 2 interrupt source
     (+) ADC_IT_AWD3: ADC Analog watchdog 3 interrupt source
     
  (#) Flags  for ADC dual mode
  (##)Flags for Master
     (+) ADC_FLAG_MSTRDY: ADC master Ready (ADRDY) flag 
     (+) ADC_FLAG_MSTEOSMP: ADC master End of Sampling flag 
     (+) ADC_FLAG_MSTEOC: ADC master End of Regular Conversion flag 
     (+) ADC_FLAG_MSTEOS: ADC master End of Regular sequence of Conversions flag 
     (+) ADC_FLAG_MSTOVR: ADC master overrun flag 
     (+) ADC_FLAG_MSTJEOC: ADC master End of Injected Conversion flag 
     (+) ADC_FLAG_MSTJEOS: ADC master End of Injected sequence of Conversions flag 
     (+) ADC_FLAG_MSTAWD1: ADC master Analog watchdog 1 flag 
     (+) ADC_FLAG_MSTAWD2: ADC master Analog watchdog 2 flag 
     (+) ADC_FLAG_MSTAWD3: ADC master Analog watchdog 3 flag 
     (+) ADC_FLAG_MSTJQOVF: ADC master Injected Context Queue Overflow flag       
     
  (##) Flags for Slave
     (+) ADC_FLAG_SLVRDY: ADC slave Ready (ADRDY) flag 
     (+) ADC_FLAG_SLVEOSMP: ADC slave End of Sampling flag 
     (+) ADC_FLAG_SLVEOC: ADC slave End of Regular Conversion flag 
     (+) ADC_FLAG_SLVEOS: ADC slave End of Regular sequence of Conversions flag 
     (+) ADC_FLAG_SLVOVR: ADC slave overrun flag 
     (+) ADC_FLAG_SLVJEOC: ADC slave End of Injected Conversion flag 
     (+) ADC_FLAG_SLVJEOS: ADC slave End of Injected sequence of Conversions flag 
     (+) ADC_FLAG_SLVAWD1: ADC slave Analog watchdog 1 flag 
     (+) ADC_FLAG_SLVAWD2: ADC slave Analog watchdog 2 flag 
     (+) ADC_FLAG_SLVAWD3: ADC slave Analog watchdog 3 flag 
     (+) ADC_FLAG_SLVJQOVF: ADC slave Injected Context Queue Overflow flag 
     
  The user should identify which mode will be used in his application to manage   
  the ADC controller events: Polling mode or Interrupt mode.
  
  In the Polling Mode it is advised to use the following functions:
      - ADC_GetFlagStatus() : to check if flags events occur. 
      - ADC_ClearFlag()     : to clear the flags events.
      
  In the Interrupt Mode it is advised to use the following functions:
     - ADC_ITConfig()       : to enable or disable the interrupt source.
     - ADC_GetITStatus()    : to check if Interrupt occurs.
     - ADC_ClearITPendingBit() : to clear the Interrupt pending Bit 
                                (corresponding Flag). 
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified ADC interrupts.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_IT: specifies the ADC interrupt sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg ADC_IT_RDY: ADC Ready (ADRDY) interrupt source 
  *     @arg ADC_IT_EOSMP: ADC End of Sampling interrupt source 
  *     @arg ADC_IT_EOC: ADC End of Regular Conversion interrupt source 
  *     @arg ADC_IT_EOS: ADC End of Regular sequence of Conversions interrupt source 
  *     @arg ADC_IT_OVR: ADC overrun interrupt source 
  *     @arg ADC_IT_JEOC: ADC End of Injected Conversion interrupt source 
  *     @arg ADC_IT_JEOS: ADC End of Injected sequence of Conversions interrupt source 
  *     @arg ADC_IT_AWD1: ADC Analog watchdog 1 interrupt source 
  *     @arg ADC_IT_AWD2: ADC Analog watchdog 2 interrupt source 
  *     @arg ADC_IT_AWD3: ADC Analog watchdog 3 interrupt source 
  *     @arg ADC_IT_JQOVF: ADC Injected Context Queue Overflow interrupt source 
  * @param  NewState: new state of the specified ADC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ITConfig(mut ADCx: *mut ADC_TypeDef,
                                      mut ADC_IT: uint32_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected ADC interrupts */
        ::core::ptr::write_volatile(&mut (*ADCx).IER as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).IER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | ADC_IT) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected ADC interrupts */
        ::core::ptr::write_volatile(&mut (*ADCx).IER as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*ADCx).IER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !ADC_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified ADC flag is set or not.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg ADC_FLAG_RDY: ADC Ready (ADRDY) flag 
  *     @arg ADC_FLAG_EOSMP: ADC End of Sampling flag 
  *     @arg ADC_FLAG_EOC: ADC End of Regular Conversion flag 
  *     @arg ADC_FLAG_EOS: ADC End of Regular sequence of Conversions flag 
  *     @arg ADC_FLAG_OVR: ADC overrun flag 
  *     @arg ADC_FLAG_JEOC: ADC End of Injected Conversion flag 
  *     @arg ADC_FLAG_JEOS: ADC End of Injected sequence of Conversions flag 
  *     @arg ADC_FLAG_AWD1: ADC Analog watchdog 1 flag 
  *     @arg ADC_FLAG_AWD2: ADC Analog watchdog 2 flag 
  *     @arg ADC_FLAG_AWD3: ADC Analog watchdog 3 flag 
  *     @arg ADC_FLAG_JQOVF: ADC Injected Context Queue Overflow flag 
  * @retval The new state of ADC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetFlagStatus(mut ADCx: *mut ADC_TypeDef,
                                           mut ADC_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified ADC flag */
    if (*ADCx).ISR & ADC_FLAG != RESET as libc::c_int as uint32_t {
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
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_FLAG: specifies the flag to clear. 
  *   This parameter can be any combination of the following values:
  *     @arg ADC_FLAG_RDY: ADC Ready (ADRDY) flag 
  *     @arg ADC_FLAG_EOSMP: ADC End of Sampling flag 
  *     @arg ADC_FLAG_EOC: ADC End of Regular Conversion flag 
  *     @arg ADC_FLAG_EOS: ADC End of Regular sequence of Conversions flag 
  *     @arg ADC_FLAG_OVR: ADC overrun flag 
  *     @arg ADC_FLAG_JEOC: ADC End of Injected Conversion flag 
  *     @arg ADC_FLAG_JEOS: ADC End of Injected sequence of Conversions flag 
  *     @arg ADC_FLAG_AWD1: ADC Analog watchdog 1 flag 
  *     @arg ADC_FLAG_AWD2: ADC Analog watchdog 2 flag 
  *     @arg ADC_FLAG_AWD3: ADC Analog watchdog 3 flag 
  *     @arg ADC_FLAG_JQOVF: ADC Injected Context Queue Overflow flag 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ClearFlag(mut ADCx: *mut ADC_TypeDef,
                                       mut ADC_FLAG: uint32_t) {
    /* Check the parameters */
    /* Clear the selected ADC flags */
    ::core::ptr::write_volatile(&mut (*ADCx).ISR as *mut uint32_t, ADC_FLAG);
}
/* *
  * @brief  Checks whether the specified ADC flag is set or not.
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_FLAG: specifies the master or slave flag to check. 
  *   This parameter can be one of the following values:
  *     @arg ADC_FLAG_MSTRDY: ADC master Ready (ADRDY) flag 
  *     @arg ADC_FLAG_MSTEOSMP: ADC master End of Sampling flag 
  *     @arg ADC_FLAG_MSTEOC: ADC master End of Regular Conversion flag 
  *     @arg ADC_FLAG_MSTEOS: ADC master End of Regular sequence of Conversions flag 
  *     @arg ADC_FLAG_MSTOVR: ADC master overrun flag 
  *     @arg ADC_FLAG_MSTJEOC: ADC master End of Injected Conversion flag 
  *     @arg ADC_FLAG_MSTJEOS: ADC master End of Injected sequence of Conversions flag 
  *     @arg ADC_FLAG_MSTAWD1: ADC master Analog watchdog 1 flag 
  *     @arg ADC_FLAG_MSTAWD2: ADC master Analog watchdog 2 flag 
  *     @arg ADC_FLAG_MSTAWD3: ADC master Analog watchdog 3 flag 
  *     @arg ADC_FLAG_MSTJQOVF: ADC master Injected Context Queue Overflow flag 
  *     @arg ADC_FLAG_SLVRDY: ADC slave Ready (ADRDY) flag 
  *     @arg ADC_FLAG_SLVEOSMP: ADC slave End of Sampling flag 
  *     @arg ADC_FLAG_SLVEOC: ADC slave End of Regular Conversion flag 
  *     @arg ADC_FLAG_SLVEOS: ADC slave End of Regular sequence of Conversions flag 
  *     @arg ADC_FLAG_SLVOVR: ADC slave overrun flag 
  *     @arg ADC_FLAG_SLVJEOC: ADC slave End of Injected Conversion flag 
  *     @arg ADC_FLAG_SLVJEOS: ADC slave End of Injected sequence of Conversions flag 
  *     @arg ADC_FLAG_SLVAWD1: ADC slave Analog watchdog 1 flag 
  *     @arg ADC_FLAG_SLVAWD2: ADC slave Analog watchdog 2 flag 
  *     @arg ADC_FLAG_SLVAWD3: ADC slave Analog watchdog 3 flag 
  *     @arg ADC_FLAG_SLVJQOVF: ADC slave Injected Context Queue Overflow flag 
  * @retval The new state of ADC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetCommonFlagStatus(mut ADCx: *mut ADC_TypeDef,
                                                 mut ADC_FLAG: uint32_t)
 -> FlagStatus {
    let mut tmpreg1: uint32_t = 0i32 as uint32_t;
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        tmpreg1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CSR
    } else {
        tmpreg1 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x700i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CSR
    }
    /* Check the status of the specified ADC flag */
    if tmpreg1 & ADC_FLAG != RESET as libc::c_int as uint32_t {
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
  * @param  ADCx: where x can be 1, 2, 3 or 4 to select the ADC peripheral.
  * @param  ADC_FLAG: specifies the master or slave flag to clear. 
  *   This parameter can be one of the following values:
  *     @arg ADC_FLAG_MSTRDY: ADC master Ready (ADRDY) flag 
  *     @arg ADC_FLAG_MSTEOSMP: ADC master End of Sampling flag 
  *     @arg ADC_FLAG_MSTEOC: ADC master End of Regular Conversion flag 
  *     @arg ADC_FLAG_MSTEOS: ADC master End of Regular sequence of Conversions flag 
  *     @arg ADC_FLAG_MSTOVR: ADC master overrun flag 
  *     @arg ADC_FLAG_MSTJEOC: ADC master End of Injected Conversion flag 
  *     @arg ADC_FLAG_MSTJEOS: ADC master End of Injected sequence of Conversions flag 
  *     @arg ADC_FLAG_MSTAWD1: ADC master Analog watchdog 1 flag 
  *     @arg ADC_FLAG_MSTAWD2: ADC master Analog watchdog 2 flag 
  *     @arg ADC_FLAG_MSTAWD3: ADC master Analog watchdog 3 flag 
  *     @arg ADC_FLAG_MSTJQOVF: ADC master Injected Context Queue Overflow flag 
  *     @arg ADC_FLAG_SLVRDY: ADC slave Ready (ADRDY) flag 
  *     @arg ADC_FLAG_SLVEOSMP: ADC slave End of Sampling flag 
  *     @arg ADC_FLAG_SLVEOC: ADC slave End of Regular Conversion flag 
  *     @arg ADC_FLAG_SLVEOS: ADC slave End of Regular sequence of Conversions flag 
  *     @arg ADC_FLAG_SLVOVR: ADC slave overrun flag 
  *     @arg ADC_FLAG_SLVJEOC: ADC slave End of Injected Conversion flag 
  *     @arg ADC_FLAG_SLVJEOS: ADC slave End of Injected sequence of Conversions flag 
  *     @arg ADC_FLAG_SLVAWD1: ADC slave Analog watchdog 1 flag 
  *     @arg ADC_FLAG_SLVAWD2: ADC slave Analog watchdog 2 flag 
  *     @arg ADC_FLAG_SLVAWD3: ADC slave Analog watchdog 3 flag 
  *     @arg ADC_FLAG_SLVJQOVF: ADC slave Injected Context Queue Overflow flag 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ClearCommonFlag(mut ADCx: *mut ADC_TypeDef,
                                             mut ADC_FLAG: uint32_t) {
    /* Check the parameters */
    if ADCx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000000i32 as
                                           libc::c_uint).wrapping_add(0i32 as
                                                                          libc::c_uint)
               as *mut ADC_TypeDef ||
           ADCx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x100i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_TypeDef {
        /* Clear the selected ADC flags */
        let ref mut fresh8 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x300i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CSR;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | ADC_FLAG) as
                                        uint32_t as uint32_t)
    } else {
        /* Clear the selected ADC flags */
        let ref mut fresh9 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x10000000i32 as
                                               libc::c_uint).wrapping_add(0x700i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut ADC_Common_TypeDef)).CSR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | ADC_FLAG) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified ADC interrupt has occurred or not.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_IT: specifies the ADC interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg ADC_IT_RDY: ADC Ready (ADRDY) interrupt source 
  *     @arg ADC_IT_EOSMP: ADC End of Sampling interrupt source 
  *     @arg ADC_IT_EOC: ADC End of Regular Conversion interrupt source 
  *     @arg ADC_IT_EOS: ADC End of Regular sequence of Conversions interrupt source 
  *     @arg ADC_IT_OVR: ADC overrun interrupt source 
  *     @arg ADC_IT_JEOC: ADC End of Injected Conversion interrupt source 
  *     @arg ADC_IT_JEOS: ADC End of Injected sequence of Conversions interrupt source 
  *     @arg ADC_IT_AWD1: ADC Analog watchdog 1 interrupt source 
  *     @arg ADC_IT_AWD2: ADC Analog watchdog 2 interrupt source 
  *     @arg ADC_IT_AWD3: ADC Analog watchdog 3 interrupt source 
  *     @arg ADC_IT_JQOVF: ADC Injected Context Queue Overflow interrupt source 
  * @retval The new state of ADC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_GetITStatus(mut ADCx: *mut ADC_TypeDef,
                                         mut ADC_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itstatus: uint16_t = 0i32 as uint16_t;
    let mut itenable: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    itstatus = ((*ADCx).ISR & ADC_IT) as uint16_t;
    itenable = ((*ADCx).IER & ADC_IT) as uint16_t;
    if itstatus as libc::c_uint != RESET as libc::c_int as uint32_t &&
           itenable as libc::c_uint != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_adc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the ADC firmware 
  *          library.
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
/* * @addtogroup ADC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  ADC Init structure definition  
  */
/* !< Specifies whether the conversion is performed in
                                               Continuous or Single mode.
                                               This parameter can be set to ENABLE or DISABLE. */
/* !< Configures the ADC resolution.
                                               This parameter can be a value of @ref ADC_resolution */
/* !< Defines the external trigger used to start the analog
                                               to digital conversion of regular channels. This parameter
                                               can be a value of @ref ADC_external_trigger_sources_for_regular_channels_conversion */
/* !< Select the external trigger edge and enable the trigger of a regular group.                                               
                                               This parameter can be a value of 
                                               @ref ADC_external_trigger_edge_for_regular_channels_conversion */
/* !< Specifies whether the ADC data alignment is left or right.
                                               This parameter can be a value of @ref ADC_data_align */
/* !< Specifies the way data overrun are managed.
                                               This parameter can be set to ENABLE or DISABLE. */
/* !< Enable/disable automatic injected group conversion after
                                               regular group conversion.
                                               This parameter can be set to ENABLE or DISABLE. */
/* !< Specifies the number of ADC channels that will be converted
                                               using the sequencer for regular channel group.
                                               This parameter must range from 1 to 16. */
/* *
  * @}
  */
/* * 
  * @brief  ADC Init structure definition  
  */
/* !< Defines the external trigger used to start the analog
                                                     to digital conversion of injected channels. This parameter
                                                     can be a value of @ref ADC_external_trigger_sources_for_Injected_channels_conversion */
/* !< Select the external trigger edge and enable the trigger of an injected group. 
                                                    This parameter can be a value of 
                                                    @ref ADC_external_trigger_edge_for_Injected_channels_conversion */
/* !< Specifies the number of ADC channels that will be converted
                                                    using the sequencer for injected channel group.
                                                    This parameter must range from 1 to 4. */
/* *
  * @}
  */
/* !< Configures the ADC to operate in 
                                               independent or multi mode. 
                                               This parameter can be a value of @ref ADC_mode */
/* !< Select the clock of the ADC. The clock is common for both master 
                                              and slave ADCs.
                                              This parameter can be a value of @ref ADC_Clock */
/* !< Configures the Direct memory access mode for multi ADC mode.                                               
                                               This parameter can be a value of 
                                               @ref ADC_Direct_memory_access_mode_for_multi_mode */
/* !< Configures the DMA mode for ADC.                                             
                                              This parameter can be a value of @ref ADC_DMA_Mode_definition */
/* !< Configures the Delay between 2 sampling phases.
                                               This parameter can be a value between  0x0 and 0xF  */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup ADC_Exported_Constants
  * @{
  */
/* * @defgroup ADC_ContinuousConvMode 
  * @{
  */
/* !<  ADC continuous conversion mode enable */
/* !<  ADC continuous conversion mode disable */
/* *
  * @}
  */
/* * @defgroup ADC_OverunMode 
  * @{
  */
/* !<  ADC Overrun Mode enable */
/* !<  ADC Overrun Mode disable */
/* *
  * @}
  */
/* * @defgroup ADC_AutoInjecMode 
  * @{
  */
/* !<  ADC Auto injected Mode enable */
/* !<  ADC Auto injected Mode disable */
/* *
  * @}
  */
/* * @defgroup ADC_resolution 
  * @{
  */
/* !<  ADC 12-bit resolution */
/* !<  ADC 10-bit resolution */
/* !<  ADC 8-bit resolution */
/* !<  ADC 6-bit resolution */
/* *
  * @}
  */
/* * @defgroup ADC_external_trigger_edge_for_regular_channels_conversion 
  * @{
  */
/* !<  ADC No external trigger for regular conversion */
/* !<  ADC external trigger rising edge for regular conversion */
/* !<  ADC ADC external trigger falling edge for regular conversion */
/* !<  ADC ADC external trigger both edges for regular conversion */
/* *
  * @}
  */
/* * @defgroup ADC_external_trigger_edge_for_Injected_channels_conversion 
  * @{
  */
/* !<  ADC No external trigger for regular conversion */
/* !<  ADC external trigger rising edge for injected conversion */
/* !<  ADC external trigger falling edge for injected conversion */
/* !<  ADC external trigger both edges for injected conversion */
/* * @defgroup ADC_external_trigger_sources_for_regular_channels_conversion 
  * @{
  */
/* !<  ADC external trigger event 0 */
/* !<  ADC external trigger event 1 */
/* !<  ADC external trigger event 2 */
/* !<  ADC external trigger event 3 */
/* !<  ADC external trigger event 4 */
/* !<  ADC external trigger event 5 */
/* !<  ADC external trigger event 6 */
/* !<  ADC external trigger event 7 */
/* !<  ADC external trigger event 8 */
/* !<  ADC external trigger event 9 */
/* !<  ADC external trigger event 10 */
/* !<  ADC external trigger event 11 */
/* !<  ADC external trigger event 12 */
/* !<  ADC external trigger event 13 */
/* !<  ADC external trigger event 14 */
/* !<  ADC external trigger event 15 */
/* *
  * @}
  */
/* * @defgroup ADC_external_trigger_sources_for_Injected_channels_conversion 
  * @{
  */
/* !<  ADC external trigger for injected conversion event 0 */
/* !<  ADC external trigger for injected conversion event 1 */
/* !<  ADC external trigger for injected conversion event 2 */
/* !<  ADC external trigger for injected conversion event 3 */
/* !<  ADC external trigger for injected conversion event 4 */
/* !<  ADC external trigger for injected conversion event 5 */
/* !<  ADC external trigger for injected conversion event 6 */
/* !<  ADC external trigger for injected conversion event 7 */
/* !<  ADC external trigger for injected conversion event 8 */
/* !<  ADC external trigger for injected conversion event 9 */
/* !<  ADC external trigger for injected conversion event 10 */
/* !<  ADC external trigger for injected conversion event 11 */
/* !<  ADC external trigger for injected conversion event 12 */
/* !<  ADC external trigger for injected conversion event 13 */
/* !<  ADC external trigger for injected conversion event 14 */
/* !<  ADC external trigger for injected conversion event 15 */
/* *
  * @}
  */
/* * @defgroup ADC_data_align 
  * @{
  */
/* !<  ADC Data alignment right */
/* !<  ADC Data alignment left */
/* *
  * @}
  */
/* * @defgroup ADC_channels 
  * @{
  */
/* !<  ADC Channel 1 */
/* !<  ADC Channel 2 */
/* !<  ADC Channel 3 */
/* !<  ADC Channel 4 */
/* !<  ADC Channel 5 */
/* !<  ADC Channel 6 */
/* !<  ADC Channel 7 */
/* !<  ADC Channel 8 */
/* !<  ADC Channel 9 */
/* !<  ADC Channel 10 */
/* !<  ADC Channel 11 */
/* !<  ADC Channel 12 */
/* !<  ADC Channel 13 */
/* !<  ADC Channel 14 */
/* !<  ADC Channel 15 */
/* !<  ADC Channel 16 */
/* !<  ADC Channel 17 */
/* !<  ADC Channel 18 */
/* *
  * @}
  */
/* * @defgroup ADC_mode 
  * @{
  */
/* !<  ADC independent mode */
/* !<  ADC multi ADC mode: Combined Regular simultaneous injected simultaneous mode */
/* !<  ADC multi ADC mode: Combined Regular simultaneous Alternate trigger mode */
/* !<  ADC multi ADC mode: Injected simultaneous mode */
/* !<  ADC multi ADC mode: Regular simultaneous mode */
/* !<  ADC multi ADC mode: Interleave mode */
/* !<  ADC multi ADC mode: Alternate Trigger mode */
/* *
  * @}
  */
/* * @defgroup ADC_Clock 
  * @{
  */
/* !< ADC Asynchronous clock mode */
/* !< Synchronous clock mode divided by 1 */
/* !<  Synchronous clock mode divided by 2 */
/* !<  Synchronous clock mode divided by 4 */
/* *
  * @}
  */
/* * @defgroup ADC_Direct_memory_access_mode_for_multi_mode 
  * @{
  */
/* !<  DMA mode disabled */
/* !<  DMA mode enabled for 12 and 10-bit resolution (6 bit) */
/* !<  DMA mode enabled for 8 and 6-bit resolution (8bit) */
/* *
  * @}
  */
/* * @defgroup ADC_sampling_time 
  * @{
  */
/* !<  ADC sampling time 1.5 cycle */
/* !<  ADC sampling time 2.5 cycles */
/* !<  ADC sampling time 4.5 cycles */
/* !<  ADC sampling time 7.5 cycles */
/* !<  ADC sampling time 19.5 cycles */
/* !<  ADC sampling time 61.5 cycles */
/* !<  ADC sampling time 181.5 cycles */
/* !<  ADC sampling time 601.5 cycles */
/* *
  * @}
  */
/* * @defgroup ADC_injected_Channel_selection 
  * @{
  */
/* !<  ADC Injected channel 1 */
/* !<  ADC Injected channel 2 */
/* !<  ADC Injected channel 3 */
/* !<  ADC Injected channel 4 */
/* !<  ADC Injected channel 5 */
/* !<  ADC Injected channel 6 */
/* !<  ADC Injected channel 7 */
/* !<  ADC Injected channel 8 */
/* !<  ADC Injected channel 9 */
/* !<  ADC Injected channel 10 */
/* !<  ADC Injected channel 11 */
/* !<  ADC Injected channel 12 */
/* !<  ADC Injected channel 13 */
/* !<  ADC Injected channel 14 */
/* !<  ADC Injected channel 15 */
/* !<  ADC Injected channel 16 */
/* !<  ADC Injected channel 17 */
/* !<  ADC Injected channel 18 */
/* *
  * @}
  */
/* * @defgroup ADC_injected_Sequence_selection 
  * @{
  */
/* !<  ADC Injected sequence 1 */
/* !<  ADC Injected sequence 2 */
/* !<  ADC Injected sequence 3 */
/* !<  ADC Injected sequence 4 */
/* *
  * @}
  */
/* * @defgroup ADC_analog_watchdog_selection 
  * @{
  */
/* !<  ADC Analog watchdog single regular mode */
/* !<  ADC Analog watchdog single injected mode */
/* !<  ADC Analog watchdog single regular or injected mode */
/* !<  ADC Analog watchdog all regular mode */
/* !<  ADC Analog watchdog all injected mode */
/* !<  ADC Analog watchdog all regular and all injected mode */
/* !<  ADC Analog watchdog off */
/* *
  * @}
  */
/* * @defgroup ADC_Calibration_Mode_definition 
  * @{
  */
/* !<  ADC Calibration for single ended channel */
/* !<  ADC Calibration for differential channel */
/* *
  * @}
  */
/* * @defgroup ADC_DMA_Mode_definition 
  * @{
  */
/* !<  ADC DMA Oneshot mode */
/* !<  ADC DMA circular mode */
/* *
  * @}
  */
/* * @defgroup ADC_interrupts_definition 
  * @{
  */
/* !< ADC Ready (ADRDY) interrupt source */
/* !< ADC End of Sampling interrupt source */
/* !< ADC End of Regular Conversion interrupt source */
/* !< ADC End of Regular sequence of Conversions interrupt source */
/* !< ADC overrun interrupt source */
/* !< ADC End of Injected Conversion interrupt source */
/* !< ADC End of Injected sequence of Conversions interrupt source */
/* !< ADC Analog watchdog 1 interrupt source */
/* !< ADC Analog watchdog 2 interrupt source */
/* !< ADC Analog watchdog 3 interrupt source */
/* !< ADC Injected Context Queue Overflow interrupt source */
/* *
  * @}
  */
/* * @defgroup ADC_flags_definition 
  * @{
  */
/* !< ADC Ready (ADRDY) flag */
/* !< ADC End of Sampling flag */
/* !< ADC End of Regular Conversion flag */
/* !< ADC End of Regular sequence of Conversions flag */
/* !< ADC overrun flag */
/* !< ADC End of Injected Conversion flag */
/* !< ADC End of Injected sequence of Conversions flag */
/* !< ADC Analog watchdog 1 flag */
/* !< ADC Analog watchdog 2 flag */
/* !< ADC Analog watchdog 3 flag */
/* !< ADC Injected Context Queue Overflow flag */
/* *
  * @}
  */
/* * @defgroup ADC_Common_flags_definition 
  * @{
  */
/* !< ADC Master Ready (ADRDY) flag */
/* !< ADC Master End of Sampling flag */
/* !< ADC Master End of Regular Conversion flag */
/* !< ADC Master End of Regular sequence of Conversions flag */
/* !< ADC Master overrun flag */
/* !< ADC Master End of Injected Conversion flag */
/* !< ADC Master End of Injected sequence of Conversions flag */
/* !< ADC Master Analog watchdog 1 flag */
/* !< ADC Master Analog watchdog 2 flag */
/* !< ADC Master Analog watchdog 3 flag */
/* !< ADC Master Injected Context Queue Overflow flag */
/* !< ADC Slave Ready (ADRDY) flag */
/* !< ADC Slave End of Sampling flag */
/* !< ADC Slave End of Regular Conversion flag */
/* !< ADC Slave End of Regular sequence of Conversions flag */
/* !< ADC Slave overrun flag */
/* !< ADC Slave End of Injected Conversion flag */
/* !< ADC Slave End of Injected sequence of Conversions flag */
/* !< ADC Slave Analog watchdog 1 flag */
/* !< ADC Slave Analog watchdog 2 flag */
/* !< ADC Slave Analog watchdog 3 flag */
/* !< ADC Slave Injected Context Queue Overflow flag */
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
/* * @defgroup ADC_regular_length 
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
/* * @defgroup ADC_two_sampling_delay_number 
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
/*  Function used to set the ADC configuration to the default reset state *****/
/* Initialization and Configuration functions *********************************/
/* Analog Watchdog configuration functions ************************************/
/* Temperature Sensor, Vrefint and Vbat management function */
/* Channels Configuration functions ***********************************/
/* Regular Channels DMA Configuration functions *******************************/
/* Injected channels Configuration functions **********************************/
/* ADC Dual Modes Configuration functions *************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the ADCx's interrupt pending bits.
  * @param  ADCx: where x can be 1, 2 or 3 to select the ADC peripheral.
  * @param  ADC_IT: specifies the ADC interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg ADC_IT_RDY: ADC Ready (ADRDY) interrupt source 
  *     @arg ADC_IT_EOSMP: ADC End of Sampling interrupt source 
  *     @arg ADC_IT_EOC: ADC End of Regular Conversion interrupt source 
  *     @arg ADC_IT_EOS: ADC End of Regular sequence of Conversions interrupt source 
  *     @arg ADC_IT_OVR: ADC overrun interrupt source 
  *     @arg ADC_IT_JEOC: ADC End of Injected Conversion interrupt source 
  *     @arg ADC_IT_JEOS: ADC End of Injected sequence of Conversions interrupt source 
  *     @arg ADC_IT_AWD1: ADC Analog watchdog 1 interrupt source 
  *     @arg ADC_IT_AWD2: ADC Analog watchdog 2 interrupt source 
  *     @arg ADC_IT_AWD3: ADC Analog watchdog 3 interrupt source 
  *     @arg ADC_IT_JQOVF: ADC Injected Context Queue Overflow interrupt source
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn ADC_ClearITPendingBit(mut ADCx: *mut ADC_TypeDef,
                                               mut ADC_IT: uint32_t) {
    /* Check the parameters */
    /* Clear the selected ADC interrupt pending bit */
    ::core::ptr::write_volatile(&mut (*ADCx).ISR as *mut uint32_t, ADC_IT);
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
