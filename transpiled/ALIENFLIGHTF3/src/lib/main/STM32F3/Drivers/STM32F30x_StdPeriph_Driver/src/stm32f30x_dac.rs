use ::libc;
extern "C" {
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DAC_InitTypeDef {
    pub DAC_Trigger: uint32_t,
    pub DAC_WaveGeneration: uint32_t,
    pub DAC_LFSRUnmask_TriangleAmplitude: uint32_t,
    pub DAC_Buffer_Switch: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup DAC_Private_Functions
  * @{
  */
/* * @defgroup DAC_Group1 DAC channels configuration
 *  @brief   DAC channels configuration: trigger, output buffer, data format 
 *
@verbatim   
 ===============================================================================
    ##### DAC channels configuration: trigger, output buffer, data format #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the DAC peripheral registers to their default reset values.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DeInit(mut DACx: *mut DAC_TypeDef) {
    /* Check the parameters */
    if DACx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7400 as libc::c_int as libc::c_uint)
               as *mut DAC_TypeDef {
        /* Enable DAC1 reset state */
        RCC_APB1PeriphResetCmd(0x20000000 as libc::c_int as uint32_t, ENABLE);
        /* Release DAC1 from reset state */
        RCC_APB1PeriphResetCmd(0x20000000 as libc::c_int as uint32_t,
                               DISABLE);
    } else {
        /* Enable DAC2 reset state */
        RCC_APB1PeriphResetCmd(0x4000000 as libc::c_int as uint32_t, ENABLE);
        /* Release DAC2 from reset state */
        RCC_APB1PeriphResetCmd(0x4000000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the DAC peripheral according to the specified 
  *         parameters in the DAC_InitStruct.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure that
  *         contains the configuration information for the specified DAC channel.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_Init(mut DACx: *mut DAC_TypeDef,
                                  mut DAC_Channel: uint32_t,
                                  mut DAC_InitStruct: *mut DAC_InitTypeDef) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the DAC parameters */
    /*---------------------------- DAC CR Configuration --------------------------*/
  /* Get the DAC CR value */
    tmpreg1 = (*DACx).CR;
    /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
    tmpreg1 &= !((0xffe as libc::c_int as uint32_t) << DAC_Channel);
    /* Configure for the selected DAC channel: buffer output, trigger, wave generation,
     mask/amplitude for wave generation */
    /* Set TSELx and TENx bits according to DAC_Trigger value */
  /* Set WAVEx bits according to DAC_WaveGeneration value */
  /* Set MAMPx bits according to DAC_LFSRUnmask_TriangleAmplitude value */ 
  /* Set BOFFx OUTENx bit according to DAC_Buffer_Switch value */
    tmpreg2 =
        (*DAC_InitStruct).DAC_Trigger | (*DAC_InitStruct).DAC_WaveGeneration |
            (*DAC_InitStruct).DAC_LFSRUnmask_TriangleAmplitude |
            (*DAC_InitStruct).DAC_Buffer_Switch;
    /* Calculate CR register value depending on DAC_Channel */
    tmpreg1 |= tmpreg2 << DAC_Channel;
    /* Write to DAC CR */
    ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Fills each DAC_InitStruct member with its default value.
  * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure which will 
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_StructInit(mut DAC_InitStruct:
                                            *mut DAC_InitTypeDef) {
    /*--------------- Reset DAC init structure parameters values -----------------*/
  /* Initialize the DAC_Trigger member */
    (*DAC_InitStruct).DAC_Trigger = 0 as libc::c_int as uint32_t;
    /* Initialize the DAC_WaveGeneration member */
    (*DAC_InitStruct).DAC_WaveGeneration = 0 as libc::c_int as uint32_t;
    /* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
    (*DAC_InitStruct).DAC_LFSRUnmask_TriangleAmplitude =
        0 as libc::c_int as uint32_t;
    /* Initialize the DAC_Buffer_Switch member */
    (*DAC_InitStruct).DAC_Buffer_Switch = 0x2 as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the specified DAC channel.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: The selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the DAC channel. 
  *          This parameter can be: ENABLE or DISABLE.
  * @note   When the DAC channel is enabled the trigger source can no more
  *         be modified.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_Cmd(mut DACx: *mut DAC_TypeDef,
                                 mut DAC_Channel: uint32_t,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DAC channel */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_int as uint32_t) <<
                                             DAC_Channel) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected DAC channel */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_int as uint32_t) <<
                                               DAC_Channel)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the selected DAC channel software trigger.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel software trigger.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SoftwareTriggerCmd(mut DACx: *mut DAC_TypeDef,
                                                mut DAC_Channel: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable software trigger for the selected DAC channel */
        ::core::ptr::write_volatile(&mut (*DACx).SWTRIGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).SWTRIGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_int as uint8_t as
                                              uint32_t) <<
                                             (DAC_Channel >>
                                                  4 as libc::c_int)) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable software trigger for the selected DAC channel */
        ::core::ptr::write_volatile(&mut (*DACx).SWTRIGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).SWTRIGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_int as uint8_t as
                                                uint32_t) <<
                                               (DAC_Channel >>
                                                    4 as libc::c_int))) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables simultaneously the two DAC channels software
  *         triggers.
  * @param  DACx: where x can be 1 to select the DAC1 peripheral.
  * @note   Dual trigger is not applicable for DAC2 (DAC2 integrates one channel).
  * @param  NewState: new state of the DAC channels software triggers.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DualSoftwareTriggerCmd(mut DACx:
                                                        *mut DAC_TypeDef,
                                                    mut NewState:
                                                        FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable software trigger for both DAC channels */
        ::core::ptr::write_volatile(&mut (*DACx).SWTRIGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).SWTRIGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x3 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable software trigger for both DAC channels */
        ::core::ptr::write_volatile(&mut (*DACx).SWTRIGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).SWTRIGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffffffc as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the selected DAC channel wave generation.
  * @param  DACx: where x can be 1 to select the DAC1 peripheral.
  * @note   Wave generation is not available in DAC2.
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_Wave: Specifies the wave type to enable or disable.
  *          This parameter can be one of the following values:
  *            @arg DAC_Wave_Noise: noise wave generation
  *            @arg DAC_Wave_Triangle: triangle wave generation
  * @param  NewState: new state of the selected DAC channel wave generation.
  *          This parameter can be: ENABLE or DISABLE.
  * @note   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_WaveGenerationCmd(mut DACx: *mut DAC_TypeDef,
                                               mut DAC_Channel: uint32_t,
                                               mut DAC_Wave: uint32_t,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected wave generation for the selected DAC channel */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         DAC_Wave << DAC_Channel) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected wave generation for the selected DAC channel */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(DAC_Wave << DAC_Channel)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Set the specified data holding register value for DAC channel1.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Align: Specifies the data alignment for DAC channel1.
  *          This parameter can be one of the following values:
  *            @arg DAC_Align_8b_R: 8bit right data alignment selected
  *            @arg DAC_Align_12b_L: 12bit left data alignment selected
  *            @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data: Data to be loaded in the selected data holding register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SetChannel1Data(mut DACx: *mut DAC_TypeDef,
                                             mut DAC_Align: uint32_t,
                                             mut Data: uint16_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, DACx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add((0x8 as
                                                                     libc::c_int
                                                                     as
                                                                     uint32_t).wrapping_add(DAC_Align))
                                    as uint32_t as uint32_t);
    /* Set the DAC channel1 selected data holding register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, Data as uint32_t);
}
/* *
  * @brief  Set the specified data holding register value for DAC channel2.
  * @param  DACx: where x can be 1 to select the DAC peripheral.
  * @note   This function is available only for DAC1.
  * @param  DAC_Align: Specifies the data alignment for DAC channel2.
  *          This parameter can be one of the following values:
  *            @arg DAC_Align_8b_R: 8bit right data alignment selected
  *            @arg DAC_Align_12b_L: 12bit left data alignment selected
  *            @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SetChannel2Data(mut DACx: *mut DAC_TypeDef,
                                             mut DAC_Align: uint32_t,
                                             mut Data: uint16_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, DACx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add((0x14 as
                                                                     libc::c_int
                                                                     as
                                                                     uint32_t).wrapping_add(DAC_Align))
                                    as uint32_t as uint32_t);
    /* Set the DAC channel2 selected data holding register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, Data as uint32_t);
}
/* *
  * @brief  Set the specified data holding register value for dual channel DAC.
  * @param  DACx: where x can be 1 to select the DAC peripheral.
  * @note   This function isn't applicable for DAC2.
  * @param  DAC_Align: Specifies the data alignment for dual channel DAC.
  *          This parameter can be one of the following values:
  *            @arg DAC_Align_8b_R: 8bit right data alignment selected
  *            @arg DAC_Align_12b_L: 12bit left data alignment selected
  *            @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data2: Data for DAC Channel2 to be loaded in the selected data 
  *         holding register.
  * @param  Data1: Data for DAC Channel1 to be loaded in the selected data 
  *         holding register.
  * @note In dual mode, a unique register access is required to write in both
  *       DAC channels at the same time.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SetDualChannelData(mut DACx: *mut DAC_TypeDef,
                                                mut DAC_Align: uint32_t,
                                                mut Data2: uint16_t,
                                                mut Data1: uint16_t) {
    let mut data: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Calculate and set dual DAC data holding register value */
    if DAC_Align == 0x8 as libc::c_int as uint32_t {
        data = (Data2 as uint32_t) << 8 as libc::c_int | Data1 as libc::c_uint
    } else {
        data =
            (Data2 as uint32_t) << 16 as libc::c_int | Data1 as libc::c_uint
    }
    tmp = DACx as uint32_t;
    tmp =
        (tmp as
             libc::c_uint).wrapping_add((0x20 as libc::c_int as
                                             uint32_t).wrapping_add(DAC_Align))
            as uint32_t as uint32_t;
    /* Set the dual DAC selected data holding register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, data);
}
/* *
  * @brief  Returns the last data output value of the selected DAC channel.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @retval The selected DAC channel data output value.
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_GetDataOutputValue(mut DACx: *mut DAC_TypeDef,
                                                mut DAC_Channel: uint32_t)
 -> uint16_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, DACx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add((0x2c as
                                                                     libc::c_int
                                                                     as
                                                                     uint32_t).wrapping_add(DAC_Channel
                                                                                                >>
                                                                                                2
                                                                                                    as
                                                                                                    libc::c_int))
                                    as uint32_t as uint32_t);
    /* Returns the DAC channel data output register value */
    return *(tmp as *mut uint32_t) as uint16_t;
}
/* *
  * @}
  */
/* * @defgroup DAC_Group2 DMA management functions
 *  @brief   DMA management functions
 *
@verbatim   
 ===============================================================================
                    ##### DMA management functions #####
 =============================================================================== 

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified DAC channel DMA request.
  *         When enabled DMA1 is generated when an external trigger (EXTI Line9,
  *         TIM2, TIM4, TIM6, TIM7 or TIM9  but not a software trigger) occurs
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel DMA request.
  *          This parameter can be: ENABLE or DISABLE.
  * @note The DAC channel1 (channel2) is mapped on DMA1 channel3 (channel4) which 
  *       must be already configured. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DMACmd(mut DACx: *mut DAC_TypeDef,
                                    mut DAC_Channel: uint32_t,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DAC channel DMA request */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1000 as libc::c_int as uint32_t)
                                             << DAC_Channel) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected DAC channel DMA request */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1000 as libc::c_int as uint32_t)
                                               << DAC_Channel)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup DAC_Group3 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions
 *
@verbatim   
 ===============================================================================
            ##### Interrupts and flags management functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified DAC interrupts.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt sources to be enabled or disabled. 
  *          This parameter can be:
  *            @arg DAC_IT_DMAUDR: DMA underrun interrupt mask
  * @note   The DMA underrun occurs when a second external trigger arrives before
  *         the acknowledgement for the first external trigger is received (first request).
  * @param  NewState: new state of the specified DAC interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_ITConfig(mut DACx: *mut DAC_TypeDef,
                                      mut DAC_Channel: uint32_t,
                                      mut DAC_IT: uint32_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DAC interrupts */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         DAC_IT << DAC_Channel) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected DAC interrupts */
        ::core::ptr::write_volatile(&mut (*DACx).CR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*DACx).CR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(DAC_IT << DAC_Channel)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief  Checks whether the specified DAC flag is set or not.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: thee selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_FLAG: specifies the flag to check. 
  *          This parameter can be:
  *            @arg DAC_FLAG_DMAUDR: DMA underrun flag
  * @note   The DMA underrun occurs when a second external trigger arrives before
  *         the acknowledgement for the first external trigger is received (first request).
  * @retval The new state of DAC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_GetFlagStatus(mut DACx: *mut DAC_TypeDef,
                                           mut DAC_Channel: uint32_t,
                                           mut DAC_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified DAC flag */
    if (*DACx).SR & DAC_FLAG << DAC_Channel !=
           RESET as libc::c_int as uint8_t as libc::c_uint {
        /* DAC_FLAG is set */
        bitstatus = SET
    } else {
        /* DAC_FLAG is reset */
        bitstatus = RESET
    }
    /* Return the DAC_FLAG status */
    return bitstatus;
}
/* *
  * @brief  Clears the DAC channel's pending flags.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_FLAG: specifies the flag to clear. 
  *          This parameter can be:
  *            @arg DAC_FLAG_DMAUDR: DMA underrun flag                          
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_ClearFlag(mut DACx: *mut DAC_TypeDef,
                                       mut DAC_Channel: uint32_t,
                                       mut DAC_FLAG: uint32_t) {
    /* Check the parameters */
    /* Clear the selected DAC flags */
    ::core::ptr::write_volatile(&mut (*DACx).SR as *mut uint32_t,
                                DAC_FLAG << DAC_Channel);
}
/* *
  * @brief  Checks whether the specified DAC interrupt has occurred or not.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.  
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt source to check. 
  *          This parameter can be:
  *            @arg DAC_IT_DMAUDR: DMA underrun interrupt mask
  * @note   The DMA underrun occurs when a second external trigger arrives before
  *         the acknowledgement for the first external trigger is received (first request).
  * @retval The new state of DAC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_GetITStatus(mut DACx: *mut DAC_TypeDef,
                                         mut DAC_Channel: uint32_t,
                                         mut DAC_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut enablestatus: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the DAC_IT enable bit status */
    enablestatus = (*DACx).CR & DAC_IT << DAC_Channel;
    /* Check the status of the specified DAC interrupt */
    if (*DACx).SR & DAC_IT << DAC_Channel != RESET as libc::c_int as uint32_t
           && enablestatus != 0 {
        /* DAC_IT is set */
        bitstatus = SET
    } else {
        /* DAC_IT is reset */
        bitstatus = RESET
    }
    /* Return the DAC_IT status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_dac.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the DAC firmware 
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
/* * @addtogroup DAC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* !< DAC channel DMA underrun interrupt enable */
/* * 
  * @brief  DAC Init structure definition
  */
/* !< Specifies the external trigger for the selected DAC channel.
                                                  This parameter can be a value of @ref DAC_trigger_selection */
/* !< Specifies whether DAC channel noise waves or triangle waves
                                                  are generated, or whether no wave is generated.
                                                  This parameter can be a value of @ref DAC_wave_generation */
/* !< Specifies the LFSR mask for noise wave generation or
                                                  the maximum amplitude triangle generation for the DAC channel. 
                                                  This parameter can be a value of @ref DAC_lfsrunmask_triangleamplitude */
/* !< Specifies whether the DAC channel output buffer is enabled or disabled or 
                                                  the DAC channel output switch is enabled or disabled.
                                                  This parameter can be a value of @ref DAC_buffer_switch */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DAC_Exported_Constants
  * @{
  */
/* * @defgroup DAC_trigger_selection 
  * @{
  */
/* !< Conversion is automatic once the DAC1_DHRxxxx register 
                                                                         has been loaded, and not by external trigger */
/* !< TIM6 TRGO selected as external conversion trigger for DAC1/2 channel1/2 */
/* !< TIM3 TRGO selected as external conversion trigger for DAC1/2 channel1/2 */
/* !< TIM8 TRGO selected as external conversion trigger for DAC1 channel1/2 */
/* !< TIM7 TRGO selected as external conversion trigger for DAC1/2 channel1/2 */
/* !< TIM15 TRGO selected as external conversion trigger for DAC1/2 channel1/2 */
/* !< HRTIM1 DACTRG1 selected as external conversion trigger for DAC1 channel1/2 */
/* !< TIM2 TRGO selected as external conversion trigger for DAC1/2 channel1/2 */
/* !< TIM4 TRGO selected as external conversion trigger for DAC channel */
/* !< HRTIM1 DACTRG2 selected as external conversion trigger for DAC1 channel1/2 */
/* !< HRTIM1 DACTRG3 selected as external conversion trigger for DAC2 channel1 */
/* !< EXTI Line9 event selected as external conversion trigger for DAC1/2 channel1/2 */
/* !< Conversion started by software trigger for DAC1/2 channel1/2 */
/* *
  * @}
  */
/* * @defgroup DAC_wave_generation 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_lfsrunmask_triangleamplitude
  * @{
  */
/* !< Unmask DAC channel LFSR bit0 for noise wave generation */
/* !< Unmask DAC channel LFSR bit[1:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[2:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[3:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[4:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[5:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[6:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[7:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[8:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[9:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[10:0] for noise wave generation */
/* !< Unmask DAC channel LFSR bit[11:0] for noise wave generation */
/* !< Select max triangle amplitude of 1 */
/* !< Select max triangle amplitude of 3 */
/* !< Select max triangle amplitude of 7 */
/* !< Select max triangle amplitude of 15 */
/* !< Select max triangle amplitude of 31 */
/* !< Select max triangle amplitude of 63 */
/* !< Select max triangle amplitude of 127 */
/* !< Select max triangle amplitude of 255 */
/* !< Select max triangle amplitude of 511 */
/* !< Select max triangle amplitude of 1023 */
/* !< Select max triangle amplitude of 2047 */
/* !< Select max triangle amplitude of 4095 */
/* *
  * @}
  */
/* * @defgroup DAC_buffer_switch 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_Channel_selection 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_data_alignement 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_wave_generation 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_data 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/*  Function used to set the DAC configuration to the default reset state *****/
/*  DAC channels configuration: trigger, output buffer, data format functions */
/* DMA management functions ***************************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the DAC channel's interrupt pending bits.
  * @param  DACx: where x can be 1 or 2 to select the DAC peripheral.
  * @param  DAC_Channel: the selected DAC channel. 
  *          This parameter can be one of the following values:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt pending bit to clear.
  *          This parameter can be the following values:
  *            @arg DAC_IT_DMAUDR: DMA underrun interrupt mask
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_ClearITPendingBit(mut DACx: *mut DAC_TypeDef,
                                               mut DAC_Channel: uint32_t,
                                               mut DAC_IT: uint32_t) {
    /* Check the parameters */
    /* Clear the selected DAC interrupt pending bits */
    ::core::ptr::write_volatile(&mut (*DACx).SR as *mut uint32_t,
                                DAC_IT << DAC_Channel);
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
