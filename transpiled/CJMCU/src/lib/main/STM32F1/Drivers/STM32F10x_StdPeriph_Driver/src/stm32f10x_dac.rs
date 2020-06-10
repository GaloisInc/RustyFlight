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
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DAC_InitTypeDef {
    pub DAC_Trigger: uint32_t,
    pub DAC_WaveGeneration: uint32_t,
    pub DAC_LFSRUnmask_TriangleAmplitude: uint32_t,
    pub DAC_OutputBuffer: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup DAC_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the DAC peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DeInit() {
    /* Enable DAC reset state */
    RCC_APB1PeriphResetCmd(0x20000000 as libc::c_int as uint32_t, ENABLE);
    /* Release DAC from reset state */
    RCC_APB1PeriphResetCmd(0x20000000 as libc::c_int as uint32_t, DISABLE);
}
/* *
  * @brief  Initializes the DAC peripheral according to the specified 
  *         parameters in the DAC_InitStruct.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure that
  *        contains the configuration information for the specified DAC channel.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_Init(mut DAC_Channel: uint32_t,
                                  mut DAC_InitStruct: *mut DAC_InitTypeDef) {
    let mut tmpreg1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the DAC parameters */
    /*---------------------------- DAC CR Configuration --------------------------*/
  /* Get the DAC CR value */
    tmpreg1 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x7400 as libc::c_int as libc::c_uint)
               as *mut DAC_TypeDef)).CR;
    /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
    tmpreg1 &= !((0xffe as libc::c_int as uint32_t) << DAC_Channel);
    /* Configure for the selected DAC channel: buffer output, trigger, wave generation,
     mask/amplitude for wave generation */
  /* Set TSELx and TENx bits according to DAC_Trigger value */
  /* Set WAVEx bits according to DAC_WaveGeneration value */
  /* Set MAMPx bits according to DAC_LFSRUnmask_TriangleAmplitude value */ 
  /* Set BOFFx bit according to DAC_OutputBuffer value */
    tmpreg2 =
        (*DAC_InitStruct).DAC_Trigger | (*DAC_InitStruct).DAC_WaveGeneration |
            (*DAC_InitStruct).DAC_LFSRUnmask_TriangleAmplitude |
            (*DAC_InitStruct).DAC_OutputBuffer;
    /* Calculate CR register value depending on DAC_Channel */
    tmpreg1 |= tmpreg2 << DAC_Channel;
    /* Write to DAC CR */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x7400 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut DAC_TypeDef)).CR as
                                    *mut uint32_t, tmpreg1);
}
/* *
  * @brief  Fills each DAC_InitStruct member with its default value.
  * @param  DAC_InitStruct : pointer to a DAC_InitTypeDef structure which will
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
    /* Initialize the DAC_OutputBuffer member */
    (*DAC_InitStruct).DAC_OutputBuffer = 0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the specified DAC channel.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the DAC channel. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_Cmd(mut DAC_Channel: uint32_t,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DAC channel */
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1 as libc::c_int as uint32_t) <<
                                             DAC_Channel) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected DAC channel */
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1 as libc::c_int as uint32_t) <<
                                               DAC_Channel)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the specified DAC channel DMA request.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel DMA request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DMACmd(mut DAC_Channel: uint32_t,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected DAC channel DMA request */
        let ref mut fresh2 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (0x1000 as libc::c_int as uint32_t)
                                             << DAC_Channel) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the selected DAC channel DMA request */
        let ref mut fresh3 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !((0x1000 as libc::c_int as uint32_t)
                                               << DAC_Channel)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the selected DAC channel software trigger.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel software trigger.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SoftwareTriggerCmd(mut DAC_Channel: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable software trigger for the selected DAC channel */
        let ref mut fresh4 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).SWTRIGR;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
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
        let ref mut fresh5 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).SWTRIGR;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
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
  *   triggers.
  * @param  NewState: new state of the DAC channels software triggers.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_DualSoftwareTriggerCmd(mut NewState:
                                                        FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable software trigger for both DAC channels */
        let ref mut fresh6 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).SWTRIGR;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x3 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable software trigger for both DAC channels */
        let ref mut fresh7 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).SWTRIGR;
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xfffffffc as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the selected DAC channel wave generation.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_Wave: Specifies the wave type to enable or disable.
  *   This parameter can be one of the following values:
  *     @arg DAC_Wave_Noise: noise wave generation
  *     @arg DAC_Wave_Triangle: triangle wave generation
  * @param  NewState: new state of the selected DAC channel wave generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_WaveGenerationCmd(mut DAC_Channel: uint32_t,
                                               mut DAC_Wave: uint32_t,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected wave generation for the selected DAC channel */
        let ref mut fresh8 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         DAC_Wave << DAC_Channel) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected wave generation for the selected DAC channel */
        let ref mut fresh9 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x7400 as libc::c_int as
                                               libc::c_uint) as
                   *mut DAC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(DAC_Wave << DAC_Channel)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Set the specified data holding register value for DAC channel1.
  * @param  DAC_Align: Specifies the data alignment for DAC channel1.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignment selected
  *     @arg DAC_Align_12b_L: 12bit left data alignment selected
  *     @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SetChannel1Data(mut DAC_Align: uint32_t,
                                             mut Data: uint16_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x7400 as
                                                                libc::c_int as
                                                                libc::c_uint));
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
  * @param  DAC_Align: Specifies the data alignment for DAC channel2.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignment selected
  *     @arg DAC_Align_12b_L: 12bit left data alignment selected
  *     @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SetChannel2Data(mut DAC_Align: uint32_t,
                                             mut Data: uint16_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x7400 as
                                                                libc::c_int as
                                                                libc::c_uint));
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
  * @brief  Set the specified data holding register value for dual channel
  *   DAC.
  * @param  DAC_Align: Specifies the data alignment for dual channel DAC.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignment selected
  *     @arg DAC_Align_12b_L: 12bit left data alignment selected
  *     @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data2: Data for DAC Channel2 to be loaded in the selected data 
  *   holding register.
  * @param  Data1: Data for DAC Channel1 to be loaded in the selected data 
  *   holding register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_SetDualChannelData(mut DAC_Align: uint32_t,
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
    tmp =
        (0x40000000 as libc::c_int as
             uint32_t).wrapping_add(0x7400 as libc::c_int as libc::c_uint);
    tmp =
        (tmp as
             libc::c_uint).wrapping_add((0x20 as libc::c_int as
                                             uint32_t).wrapping_add(DAC_Align))
            as uint32_t as uint32_t;
    /* Set the dual DAC selected data holding register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, data);
}
/* *
  ******************************************************************************
  * @file    stm32f10x_dac.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the DAC firmware 
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
/* * @addtogroup DAC
  * @{
  */
/* * @defgroup DAC_Exported_Types
  * @{
  */
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
/* !< Specifies whether the DAC channel output buffer is enabled or disabled.
                                                  This parameter can be a value of @ref DAC_output_buffer */
/* *
  * @}
  */
/* * @defgroup DAC_Exported_Constants
  * @{
  */
/* * @defgroup DAC_trigger_selection 
  * @{
  */
/* !< Conversion is automatic once the DAC1_DHRxxxx register 
                                                                       has been loaded, and not by external trigger */
/* !< TIM6 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM8 TRGO selected as external conversion trigger for DAC channel
                                                                       only in High-density devices*/
/* !< TIM8 TRGO selected as external conversion trigger for DAC channel
                                                                       only in Connectivity line, Medium-density and Low-density Value Line devices */
/* !< TIM7 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM5 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM15 TRGO selected as external conversion trigger for DAC channel 
                                                                       only in Medium-density and Low-density Value Line devices*/
/* !< TIM2 TRGO selected as external conversion trigger for DAC channel */
/* !< TIM4 TRGO selected as external conversion trigger for DAC channel */
/* !< EXTI Line9 event selected as external conversion trigger for DAC channel */
/* !< Conversion started by software trigger for DAC channel */
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
/* * @defgroup DAC_output_buffer 
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
/* * @defgroup DAC_data_alignment 
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
/* *
  * @}
  */
/* * @defgroup DAC_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup DAC_Exported_Functions
  * @{
  */
/* *
  * @brief  Returns the last data output value of the selected DAC channel.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @retval The selected DAC channel data output value.
  */
#[no_mangle]
pub unsafe extern "C" fn DAC_GetDataOutputValue(mut DAC_Channel: uint32_t)
 -> uint16_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x7400 as
                                                                libc::c_int as
                                                                libc::c_uint));
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
