use ::libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct COMP_InitTypeDef {
    pub COMP_InvertingInput: uint32_t,
    pub COMP_NonInvertingInput: uint32_t,
    pub COMP_Output: uint32_t,
    pub COMP_BlankingSrce: uint32_t,
    pub COMP_OutputPol: uint32_t,
    pub COMP_Hysteresis: uint32_t,
    pub COMP_Mode: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup COMP_Private_Functions
  * @{
  */
/* * @defgroup COMP_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim   
 ===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes COMP peripheral registers to their default reset values.
  * @note   Deinitialization can't be performed if the COMP configuration is locked.
  *         To unlock the configuration, perform a system reset.
  * @param  COMP_Selection: the selected comparator. 
  *          This parameter can be COMP_Selection_COMPx where x can be 1 to 7
  *          to select the COMP peripheral.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_DeInit(mut COMP_Selection: uint32_t) {
    /* !< Set COMP_CSR register to reset value */
    ::core::ptr::write_volatile((0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x10000 as
                                                                libc::c_int as
                                                                libc::c_uint).wrapping_add(0x1c
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(COMP_Selection)
                                    as *mut uint32_t,
                                0 as libc::c_int as uint32_t);
}
/* *
  * @brief  Initializes the COMP peripheral according to the specified parameters
  *         in COMP_InitStruct
  * @note   If the selected comparator is locked, initialization can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @note   By default, PA1 is selected as COMP1 non inverting input.
  *         To use PA4 as COMP1 non inverting input call COMP_SwitchCmd() after COMP_Init()
  * @param  COMP_Selection: the selected comparator. 
  *          This parameter can be COMP_Selection_COMPx where x can be 1 to 7
  *          to select the COMP peripheral.
  * @param  COMP_InitStruct: pointer to an COMP_InitTypeDef structure that contains 
  *         the configuration information for the specified COMP peripheral.
  *           - COMP_InvertingInput specifies the inverting input of COMP
  *           - COMP_NonInvertingInput specifies the non inverting input of COMP
  *           - COMP_Output connect COMP output to selected timer
  *             input (Input capture / Output Compare Reference Clear / Break Input)
  *           - COMP_BlankingSrce specifies the blanking source of COMP
  *           - COMP_OutputPol select output polarity
  *           - COMP_Hysteresis configures COMP hysteresis value
  *           - COMP_Mode configures COMP power mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_Init(mut COMP_Selection: uint32_t,
                                   mut COMP_InitStruct:
                                       *mut COMP_InitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* !< Get the COMPx_CSR register value */
    tmpreg =
        *((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x10000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x1c as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_add(COMP_Selection)
              as *mut uint32_t);
    /* !< Clear the COMP1SW1, COMPxINSEL, COMPxOUTSEL, COMPxPOL, COMPxHYST and COMPxMODE bits */
    tmpreg &= 0x3 as libc::c_int as uint32_t;
    /* !< Configure COMP: inverting input, output redirection, hysteresis value and power mode */
  /* !< Set COMPxINSEL bits according to COMP_InitStruct->COMP_InvertingInput value */
  /* !< Set COMPxNONINSEL bits according to COMP_InitStruct->COMP_NonInvertingInput value */
  /* !< Set COMPxBLANKING bits according to COMP_InitStruct->COMP_BlankingSrce value */
  /* !< Set COMPxOUTSEL bits according to COMP_InitStruct->COMP_Output value */
  /* !< Set COMPxPOL bit according to COMP_InitStruct->COMP_OutputPol value */
  /* !< Set COMPxHYST bits according to COMP_InitStruct->COMP_Hysteresis value */
  /* !< Set COMPxMODE bits according to COMP_InitStruct->COMP_Mode value */
    tmpreg |=
        (*COMP_InitStruct).COMP_InvertingInput |
            (*COMP_InitStruct).COMP_NonInvertingInput |
            (*COMP_InitStruct).COMP_Output | (*COMP_InitStruct).COMP_OutputPol
            | (*COMP_InitStruct).COMP_BlankingSrce |
            (*COMP_InitStruct).COMP_Hysteresis | (*COMP_InitStruct).COMP_Mode;
    /* !< Write to COMPx_CSR register */
    ::core::ptr::write_volatile((0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add(0x10000 as
                                                                libc::c_int as
                                                                libc::c_uint).wrapping_add(0x1c
                                                                                               as
                                                                                               libc::c_int
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(COMP_Selection)
                                    as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Fills each COMP_InitStruct member with its default value.
  * @param  COMP_InitStruct: pointer to an COMP_InitTypeDef structure which will 
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_StructInit(mut COMP_InitStruct:
                                             *mut COMP_InitTypeDef) {
    (*COMP_InitStruct).COMP_InvertingInput = 0 as libc::c_int as uint32_t;
    (*COMP_InitStruct).COMP_NonInvertingInput = 0 as libc::c_int as uint32_t;
    (*COMP_InitStruct).COMP_Output = 0 as libc::c_int as uint32_t;
    (*COMP_InitStruct).COMP_BlankingSrce = 0 as libc::c_int as uint32_t;
    (*COMP_InitStruct).COMP_OutputPol = 0 as libc::c_int as uint32_t;
    (*COMP_InitStruct).COMP_Hysteresis = 0 as libc::c_int as uint32_t;
    (*COMP_InitStruct).COMP_Mode = 0xc as libc::c_int as uint32_t;
}
/* *
  * @brief  Enable or disable the COMP peripheral.
  * @note   If the selected comparator is locked, enable/disable can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  COMP_Selection: the selected comparator. 
  *          This parameter can be COMP_Selection_COMPx where x can be 1 to 7
  *          to select the COMP peripheral.
  * @param  NewState: new state of the COMP peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  *         When enabled, the comparator compares the non inverting input with 
  *                       the inverting input and the comparison result is available
  *                       on comparator output.
  *         When disabled, the comparator doesn't perform comparison and the 
  *                        output level is low.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_Cmd(mut COMP_Selection: uint32_t,
                                  mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected COMPx peripheral */
        let ref mut fresh0 =
            *((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1c
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_add(COMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected COMP peripheral  */
        let ref mut fresh1 =
            *((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1c
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_add(COMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Close or Open the SW1 switch.
  * @note   If the COMP1 is locked, Close/Open the SW1 switch can't be performed.
  *         To unlock the configuration, perform a system reset.  
  * @note   This switch is solely intended to redirect signals onto high
  *         impedance input, such as COMP1 non-inverting input (highly resistive switch)
  * @param  NewState: New state of the analog switch.
  *   This parameter can be 
  *     ENABLE so the SW1 is closed; PA1 is connected to PA4
  *     or DISABLE so the SW1 switch is open; PA1 is disconnected from PA4
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_SwitchCmd(mut COMP_Selection: uint32_t,
                                        mut NewState: FunctionalState) {
    /* Check the parameter */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Close SW1 switch */
        let ref mut fresh2 =
            *((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1c
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_add(COMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Open SW1 switch */
        let ref mut fresh3 =
            *((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1c
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_add(COMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Return the output level (high or low) of the selected comparator. 
  *         The output level depends on the selected polarity.
  *         If the polarity is not inverted:
  *           - Comparator output is low when the non-inverting input is at a lower
  *             voltage than the inverting input
  *           - Comparator output is high when the non-inverting input is at a higher
  *             voltage than the inverting input
  *         If the polarity is inverted:
  *           - Comparator output is high when the non-inverting input is at a lower
  *             voltage than the inverting input
  *           - Comparator output is low when the non-inverting input is at a higher
  *             voltage than the inverting input
  * @param  COMP_Selection: the selected comparator. 
  *          This parameter can be COMP_Selection_COMPx where x can be 1 to 7
  *          to select the COMP peripheral.
  * @retval Returns the selected comparator output level: low or high.
  *       
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_GetOutputLevel(mut COMP_Selection: uint32_t)
 -> uint32_t {
    let mut compout: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Check if selected comparator output is high */
    if *((0x40000000 as libc::c_int as
              uint32_t).wrapping_add(0x10000 as libc::c_int as
                                         libc::c_uint).wrapping_add(0x1c as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(COMP_Selection)
             as *mut uint32_t) & 0x40000000 as libc::c_int as uint32_t !=
           0 as libc::c_int as libc::c_uint {
        compout = 0x40000000 as libc::c_int as uint32_t
    } else { compout = 0 as libc::c_int as uint32_t }
    /* Return the comparator output level */
    return compout;
}
/* *
  * @}
  */
/* * @defgroup COMP_Group2 Window mode control function
 *  @brief   Window mode control function 
 *
@verbatim   
 ===============================================================================
                    ##### Window mode control function #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the window mode.
  *         Window mode for comparators makes use of two comparators:
  *         COMP1 and COM2, COMP3 and COMP4, COMP5 and COMP6.
  *         In window mode, COMPx and COMPx-1 (where x can be 2, 4 or 6)
  *         non inverting inputs are connected together and only COMPx-1 non
  *         inverting input can be used.
  *         e.g When window mode enabled for COMP4, COMP3 non inverting input (PB14 or PD14)
  *             is to be used.
  * @note   If the COMPx is locked, ENABLE/DISABLE the window mode can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  COMP_Selection: the selected comparator.
  *          This parameter can be COMP_Selection_COMPx where x can be 2, 4 or 6
  *          to select the COMP peripheral.
  * param   NewState: new state of the window mode.
  *   This parameter can be ENABLE or DISABLE.
  *        When enbaled, COMPx and COMPx-1 non inverting inputs are connected together.
  *        When disabled, COMPx and COMPx-1 non inverting inputs are disconnected.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_WindowCmd(mut COMP_Selection: uint32_t,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the window mode */
        let ref mut fresh4 =
            *((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1c
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_add(COMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x200 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the window mode */
        let ref mut fresh5 =
            *((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x10000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x1c
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint).wrapping_add(COMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x200 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  ******************************************************************************
  * @file    stm32f30x_comp.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the COMP firmware 
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
/* * @addtogroup COMP
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  COMP Init structure definition  
  */
/* !< Selects the inverting input of the comparator.
                                          This parameter can be a value of @ref COMP_InvertingInput */
/* !< Selects the non inverting input of the comparator.
                                         This parameter can be a value of @ref COMP_NonInvertingInput */
/* !< Selects the output redirection of the comparator.
                                          This parameter can be a value of @ref COMP_Output */
/* !< Selects the output blanking source of the comparator.
                                          This parameter can be a value of @ref COMP_BlankingSrce */
/* !< Selects the output polarity of the comparator.
                                          This parameter can be a value of @ref COMP_OutputPoloarity */
/* !< Selects the hysteresis voltage of the comparator.
                                          This parameter can be a value of @ref COMP_Hysteresis */
/* !< Selects the operating mode of the comparator
                                         and allows to adjust the speed/consumption.
                                         This parameter can be a value of @ref COMP_Mode */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup COMP_Exported_Constants
  * @{
  */
/* * @defgroup COMP_Selection
  * @{
  */
/* !< COMP1 Selection */
/* !< COMP2 Selection */
/* !< COMP3 Selection */
/* !< COMP4 Selection */
/* !< COMP5 Selection */
/* !< COMP6 Selection */
/* !< COMP7 Selection */
/* *
  * @}
  */
/* * @defgroup COMP_InvertingInput
  * @{
  */
/* !< 1/4 VREFINT connected to comparator inverting input */
/* !< 1/2 VREFINT connected to comparator inverting input */
/* !< 3/4 VREFINT connected to comparator inverting input */
/* !< VREFINT connected to comparator inverting input */
/* !< DAC1_OUT1 (PA4) connected to comparator inverting input */
/* !< DAC1_OUT2 (PA5) connected to comparator inverting input */
/* !< I/O1 (PA0 for COMP1, PA2 for COMP2, PD15 for COMP3, 
                                                                            PE8 for COMP4, PD13 for COMP5, PD10 for COMP6,
                                                                            PC0 for COMP7) connected to comparator inverting input */
/* !< I/O2 (PB12 for COMP3, PB2 for COMP4, PB10 for COMP5,
                                                                            PB15 for COMP6) connected to comparator inverting input */
/* !< DAC2_OUT1 (PA6) connected to comparator inverting input */
/* *
  * @}
  */
/* * @defgroup COMP_NonInvertingInput
  * @{
  */
/* !< I/O1 (PA1 for COMP1, PA7 for COMP2, PB14 for COMP3, 
                                                                               PB0 for COMP4, PD12 for COMP5, PD11 for COMP6,
                                                                               PA0 for COMP7) connected to comparator non inverting input */
/* !< I/O2 (PA3 for COMP2, PD14 for COMP3, PE7 for COMP4, PB13 for COMP5,
                                                                               PB11 for COMP6, PC1 for COMP7) connected to comparator non inverting input */
/* *
  * @}
  */
/* * @defgroup COMP_Output
  * @{
  */
/* !< COMP output isn't connected to other peripherals */
/* Output Redirection common for all comparators COMP1...COMP7 */
/* !< COMP output connected to TIM1 Break Input (BKIN) */
/* !< COMP output connected to TIM1 Break Input 2 (BKIN2) */
/* !< COMP output connected to TIM8 Break Input (BKIN) */
/* !< COMP output connected to TIM8 Break Input 2 (BKIN2) */
/* !< COMP output connected to TIM1 Break Input 2 and TIM8 Break Input 2 */
/* Output Redirection common for COMP1 and COMP2 */
/* !< COMP output connected to TIM1 OCREF Clear */
/* !< COMP output connected to TIM1 Input Capture 1 */
/* !< COMP output connected to TIM2 Input Capture 4 */
/* !< COMP output connected to TIM2 OCREF Clear */
/* !< COMP output connected to TIM3 Input Capture 1 */
/* !< COMP output connected to TIM3 OCREF Clear */
/* Output Redirection specific to COMP2 */
/* !< COMP output connected to HRTIM1 FLT6 */
/* !< COMP output connected to HRTIM1 EE1_2*/
/* !< COMP output connected to HRTIM1 EE6_2 */
/* Output Redirection specific to COMP3 */
/* !< COMP output connected to TIM4 Input Capture 1 */
/* !< COMP output connected to TIM3 Input Capture 2 */
/* !< COMP output connected to TIM15 Input Capture 1 */
/* !< COMP output connected to TIM15 Break Input (BKIN) */
/* Output Redirection specific to COMP4 */
/* !< COMP output connected to TIM3 Input Capture 3 */
/* !< COMP output connected to TIM8 OCREF Clear */
/* !< COMP output connected to TIM15 Input Capture 2 */
/* !< COMP output connected to TIM4 Input Capture 2 */
/* !< COMP output connected to TIM15 OCREF Clear */
/* !< COMP output connected to HRTIM1 FLT7 */
/* !< COMP output connected to HRTIM1 EE2_2*/
/* !< COMP output connected to HRTIM1 EE7_2 */
/* Output Redirection specific to COMP5 */
/* !< COMP output connected to TIM2 Input Capture 1 */
/* !< COMP output connected to TIM17 Input Capture 1 */
/* !< COMP output connected to TIM4 Input Capture 3 */
/* !< COMP output connected to TIM16 Break Input (BKIN) */
/* Output Redirection specific to COMP6 */
/* !< COMP output connected to TIM2 Input Capture 2 */
/* !< COMP output connected to TIM2 OCREF Clear */
/* !< COMP output connected to TIM16 OCREF Clear */
/* !< COMP output connected to TIM16 Input Capture 1 */
/* !< COMP output connected to TIM4 Input Capture 4 */
/* !< COMP output connected to HRTIM1 FLT8 */
/* !< COMP output connected to HRTIM1 EE3_2*/
/* !< COMP output connected to HRTIM1 EE8_2 */
/* Output Redirection specific to COMP7 */
/* !< COMP output connected to TIM2 Input Capture 3 */
/* !< COMP output connected to TIM1 Input Capture 2 */
/* !< COMP output connected to TIM16 OCREF Clear */
/* !< COMP output connected to TIM16 Break Input (BKIN) */
/* *
  * @}
  */
/* * @defgroup COMP_BlankingSrce
  * @{
  */
/* No blanking source can be selected for all comparators */
/* !< No blanking source */
/* Blanking source common for COMP1, COMP2, COMP3 and COMP7 */
/* !< TIM1 OC5 selected as blanking source for compartor */
/* Blanking source common for COMP1 and COMP2 */
/* !< TIM2 OC5 selected as blanking source for compartor */
/* Blanking source common for COMP1, COMP2 and COMP5 */
/* !< TIM2 OC3 selected as blanking source for compartor */
/* Blanking source common for COMP3 and COMP6 */
/* !< TIM2 OC4 selected as blanking source for compartor */
/* Blanking source common for COMP4, COMP5, COMP6 and COMP7 */
/* !< TIM8 OC5 selected as blanking source for compartor */
/* Blanking source for COMP4 */
/* !< TIM3 OC4 selected as blanking source for compartor */
/* !< TIM15 OC1 selected as blanking source for compartor */
/* Blanking source common for COMP6 and COMP7 */
/* !< TIM15 OC2 selected as blanking source for compartor */
/* *
  * @}
  */
/* * @defgroup COMP_OutputPoloarity
  * @{
  */
/* !< COMP output on GPIO isn't inverted */
/* !< COMP output on GPIO is inverted */
/* *
  * @}
  */
/* * @defgroup COMP_Hysteresis
  * @{
  */
/* Please refer to the electrical characteristics in the device datasheet for
   the hysteresis level */
/* !< No hysteresis */
/* !< Hysteresis level low */
/* !< Hysteresis level medium */
/* !< Hysteresis level high */
/* *
  * @}
  */
/* * @defgroup COMP_Mode
  * @{
  */
/* Please refer to the electrical characteristics in the device datasheet for
   the power consumption values */
/* !< High Speed */
/* !< Medium Speed */
/* !< Low power mode */
/* !< Ultra-low power mode */
/* *
  * @}
  */
/* * @defgroup COMP_OutputLevel
  * @{
  */ 
/* When output polarity is not inverted, comparator output is high when
   the non-inverting input is at a higher voltage than the inverting input */
/* When output polarity is not inverted, comparator output is low when
   the non-inverting input is at a lower voltage than the inverting input*/
/* *
  * @}
  */
/* * @defgroup COMP_WindowMode
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
/*  Function used to set the COMP configuration to the default reset state ****/
/* Initialization and Configuration functions *********************************/
/* Window mode control function ***********************************************/
/* COMP configuration locking function ****************************************/
/* *
  * @}
  */
/* * @defgroup COMP_Group3 COMP configuration locking function
 *  @brief   COMP1, COMP2,...COMP7 configuration locking function
 *           COMP1, COMP2,...COMP7 configuration can be locked each separately.
 *           Unlocking is performed by system reset.
 *
@verbatim   
 ===============================================================================
                   ##### Configuration Lock function #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Lock the selected comparator (COMP1/COMP2) configuration.
  * @note   Locking the configuration means that all control bits are read-only.
  *         To unlock the comparator configuration, perform a system reset.
  * @param  COMP_Selection: the selected comparator. 
  *          This parameter can be COMP_Selection_COMPx where x can be 1 to 7
  *          to select the COMP peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn COMP_LockConfig(mut COMP_Selection: uint32_t) {
    /* Check the parameter */
    /* Set the lock bit corresponding to selected comparator */
    let ref mut fresh6 =
        *((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x10000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x1c as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint).wrapping_add(COMP_Selection)
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x80000000 as libc::c_uint) as uint32_t
                                    as uint32_t);
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
