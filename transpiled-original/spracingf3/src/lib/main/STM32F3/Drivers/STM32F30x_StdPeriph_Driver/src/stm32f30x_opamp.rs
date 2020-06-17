use core;
use libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct OPAMP_InitTypeDef {
    pub OPAMP_InvertingInput: uint32_t,
    pub OPAMP_NonInvertingInput: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup OPAMP_Private_Functions
  * @{
  */
/* * @defgroup OPAMP_Group1 Initialization and Configuration functions
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
  * @brief  Deinitializes OPAMP peripheral registers to their default reset values.
  * @note   Deinitialization can't be performed if the OPAMP configuration is locked.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_DeInit(mut OPAMP_Selection: uint32_t) {
    /* !< Set OPAMP_CSR register to reset value */
    ::core::ptr::write_volatile((0x40000000i32 as
                                     uint32_t).wrapping_add(0x10000i32 as
                                                                libc::c_uint).wrapping_add(0x38i32
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(OPAMP_Selection)
                                    as *mut uint32_t, 0i32 as uint32_t);
}
/* *
  * @brief  Initializes the OPAMP peripheral according to the specified parameters
  *         in OPAMP_InitStruct
  * @note   If the selected OPAMP is locked, initialization can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  OPAMP_InitStruct: pointer to an OPAMP_InitTypeDef structure that contains 
  *         the configuration information for the specified OPAMP peripheral.
  *           - OPAMP_InvertingInput specifies the inverting input of OPAMP
  *           - OPAMP_NonInvertingInput specifies the non inverting input of OPAMP
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_Init(mut OPAMP_Selection: uint32_t,
                                    mut OPAMP_InitStruct:
                                        *mut OPAMP_InitTypeDef) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* !< Get the OPAMPx_CSR register value */
    tmpreg =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    /* !< Clear the inverting and non inverting bits selection bits */
    tmpreg &= 0xffffff93u32;
    /* !< Configure OPAMP: inverting and non inverting inputs */
    tmpreg |=
        (*OPAMP_InitStruct).OPAMP_InvertingInput |
            (*OPAMP_InitStruct).OPAMP_NonInvertingInput;
    /* !< Write to OPAMPx_CSR register */
    ::core::ptr::write_volatile((0x40000000i32 as
                                     uint32_t).wrapping_add(0x10000i32 as
                                                                libc::c_uint).wrapping_add(0x38i32
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(OPAMP_Selection)
                                    as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Fills each OPAMP_InitStruct member with its default value.
  * @param  OPAMP_InitStruct: pointer to an OPAMP_InitTypeDef structure which will 
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_StructInit(mut OPAMP_InitStruct:
                                              *mut OPAMP_InitTypeDef) {
    (*OPAMP_InitStruct).OPAMP_NonInvertingInput = 0i32 as uint32_t;
    (*OPAMP_InitStruct).OPAMP_InvertingInput = 0i32 as uint32_t;
}
/* *
  * @brief  Configure the feedback resistor gain.
  * @note   If the selected OPAMP is locked, gain configuration can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  NewState: new state of the OPAMP peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_PGAConfig(mut OPAMP_Selection: uint32_t,
                                         mut OPAMP_PGAGain: uint32_t,
                                         mut OPAMP_PGAConnect: uint32_t) {
    /* Check the parameters */
    /* Reset the configuration bits */
    let ref mut fresh0 =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x3c000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Set the new configuration */
    let ref mut fresh1 =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (OPAMP_PGAGain | OPAMP_PGAConnect)) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configure the OPAMP's internal reference.
  * @note   This feature is used when calibration enabled or OPAMP's reference
  *         connected to the non inverting input.
  * @note   If the selected OPAMP is locked, Vref configuration can't be performed.
  *         To unlock the configuration, perform a system reset.  
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  OPAMP_Vref: This parameter can be:
  *           OPAMP_Vref_3VDDA: OPMAP Vref = 3.3% VDDA
  *           OPAMP_Vref_10VDDA: OPMAP Vref = 10% VDDA
  *           OPAMP_Vref_50VDDA: OPMAP Vref = 50% VDDA
  *           OPAMP_Vref_90VDDA: OPMAP Vref = 90% VDDA
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_VrefConfig(mut OPAMP_Selection: uint32_t,
                                          mut OPAMP_Vref: uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* !< Get the OPAMPx_CSR register value */
    tmpreg =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    /* !< Clear the CALSEL bits */
    tmpreg &= !(0x3000i32 as uint32_t);
    /* !< Configure OPAMP reference */
    tmpreg |= OPAMP_Vref;
    /* !< Write to OPAMPx_CSR register */
    ::core::ptr::write_volatile((0x40000000i32 as
                                     uint32_t).wrapping_add(0x10000i32 as
                                                                libc::c_uint).wrapping_add(0x38i32
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(OPAMP_Selection)
                                    as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Connnect the internal reference to the OPAMP's non inverting input.
  * @note   If the selected OPAMP is locked, Vref configuration can't be performed.
  *         To unlock the configuration, perform a system reset.  
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  NewState: new state of the OPAMP peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_VrefConnectNonInvertingInput(mut OPAMP_Selection:
                                                                uint32_t,
                                                            mut NewState:
                                                                FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Connnect the internal reference to the OPAMP's non inverting input */
        let ref mut fresh2 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x2i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disconnnect the internal reference to the OPAMP's non inverting input */
        let ref mut fresh3 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2i32 as uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables connecting the OPAMP's internal reference to ADC.
  * @note   If the selected OPAMP is locked, Vref connection can't be performed.
  *         To unlock the configuration, perform a system reset.  
  * @param  NewState: new state of the Vrefint output.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_VrefConnectADCCmd(mut OPAMP_Selection:
                                                     uint32_t,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable output internal reference */
        let ref mut fresh4 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20000000i32 as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable output internal reference */
        let ref mut fresh5 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x20000000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configure the OPAMP peripheral (secondary inputs) for timer-controlled
  *          mux mode according to the specified parameters in OPAMP_InitStruct.
  * @note   If the selected OPAMP is locked, timer-controlled mux configuration
  *         can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  OPAMP_InitStruct: pointer to an OPAMP_InitTypeDef structure that contains 
  *         the configuration information for the specified OPAMP peripheral.
  *           - OPAMP_InvertingInput specifies the inverting input of OPAMP
  *           - OPAMP_NonInvertingInput specifies the non inverting input of OPAMP
  * @note   PGA and Vout can't be selected as seconadry inverting input.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_TimerControlledMuxConfig(mut OPAMP_Selection:
                                                            uint32_t,
                                                        mut OPAMP_InitStruct:
                                                            *mut OPAMP_InitTypeDef) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* !< Get the OPAMPx_CSR register value */
    tmpreg =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    /* !< Clear the secondary inverting bit, secondary non inverting bit and TCMEN bits */
    tmpreg &= 0xfffff8ffu32;
    /* !< Configure OPAMP: secondary inverting and non inverting inputs */
    tmpreg |=
        (*OPAMP_InitStruct).OPAMP_InvertingInput << 3i32 |
            (*OPAMP_InitStruct).OPAMP_NonInvertingInput << 7i32;
    /* !< Write to OPAMPx_CSR register */
    ::core::ptr::write_volatile((0x40000000i32 as
                                     uint32_t).wrapping_add(0x10000i32 as
                                                                libc::c_uint).wrapping_add(0x38i32
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(OPAMP_Selection)
                                    as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enable or disable the timer-controlled mux mode.
  * @note   If the selected OPAMP is locked, enable/disable can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  NewState: new state of the OPAMP peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_TimerControlledMuxCmd(mut OPAMP_Selection:
                                                         uint32_t,
                                                     mut NewState:
                                                         FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the timer-controlled Mux mode */
        let ref mut fresh6 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the timer-controlled Mux mode */
        let ref mut fresh7 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh7,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief  Enable or disable the OPAMP peripheral.
  * @note   If the selected OPAMP is locked, enable/disable can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  NewState: new state of the OPAMP peripheral.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_Cmd(mut OPAMP_Selection: uint32_t,
                                   mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected OPAMPx peripheral */
        let ref mut fresh8 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh8,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x1i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable the selected OPAMPx peripheral */
        let ref mut fresh9 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1i32 as uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Return the output level (high or low) during calibration of the selected OPAMP. 
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  *           - OPAMP output is low when the non-inverting input is at a lower
  *             voltage than the inverting input
  *           - OPAMP output is high when the non-inverting input is at a higher
  *             voltage than the inverting input
  * @note OPAMP ouput level is provided only during calibration phase.
  * @retval Returns the selected OPAMP output level: low or high.
  *       
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_GetOutputLevel(mut OPAMP_Selection: uint32_t)
 -> uint32_t {
    let mut opampout: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Check if selected OPAMP output is high */
    if *((0x40000000i32 as
              uint32_t).wrapping_add(0x10000i32 as
                                         libc::c_uint).wrapping_add(0x38i32 as
                                                                        libc::c_uint).wrapping_add(OPAMP_Selection)
             as *mut uint32_t) & 0x40000000i32 as uint32_t !=
           0i32 as libc::c_uint {
        opampout = 0x40000000i32 as uint32_t
    } else { opampout = 0i32 as uint32_t }
    /* Return the OPAMP output level */
    return opampout;
}
/* *
  * @brief  Select the trimming mode.
  * @param  OffsetTrimming: the selected offset trimming mode. 
  *   This parameter  can be one of the following values:
  *     @arg OPAMP_Trimming_Factory: factory trimming values are used for offset
  *                                  calibration
  *     @arg OPAMP_Trimming_User: user trimming values are used for offset
  *                               calibration
  * @note When OffsetTrimming_User is selected, use OPAMP_OffsetTrimConfig()
  *       function or OPAMP_OffsetTrimLowPowerConfig() function to adjust 
  *       trimming value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_OffsetTrimModeSelect(mut OPAMP_Selection:
                                                        uint32_t,
                                                    mut OPAMP_Trimming:
                                                        uint32_t) {
    /* Check the parameters */
    /* Reset USERTRIM bit */
    let ref mut fresh10 =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x40000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Select trimming mode */
    let ref mut fresh11 =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OPAMP_Trimming) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configure the trimming value of the OPAMP.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  OPAMP_Input: the selected OPAMP input. 
  *   This parameter can be one of the following values:
  *         @arg OPAMP_Input_Inverting: Inverting input is selected to configure the trimming value
  *         @arg OPAMP_Input_NonInverting: Non inverting input is selected to configure the trimming value
  * @param  OPAMP_TrimValue: the trimming value. This parameter can be any value lower
  *         or equal to 0x0000001F. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_OffsetTrimConfig(mut OPAMP_Selection: uint32_t,
                                                mut OPAMP_Input: uint32_t,
                                                mut OPAMP_TrimValue:
                                                    uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* !< Get the OPAMPx_CSR register value */
    tmpreg =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    /* !< Clear the trimming bits */
    tmpreg &= !((0x1fi32 as uint32_t) << OPAMP_Input);
    /* !< Configure the new trimming value */
    tmpreg |= OPAMP_TrimValue << OPAMP_Input;
    /* !< Write to OPAMPx_CSR register */
    ::core::ptr::write_volatile((0x40000000i32 as
                                     uint32_t).wrapping_add(0x10000i32 as
                                                                libc::c_uint).wrapping_add(0x38i32
                                                                                               as
                                                                                               libc::c_uint).wrapping_add(OPAMP_Selection)
                                    as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Start or stop the calibration of selected OPAMP peripheral.
  * @note   If the selected OPAMP is locked, start/stop can't be performed.
  *         To unlock the configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @param  NewState: new state of the OPAMP peripheral.
  *         This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_StartCalibration(mut OPAMP_Selection: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Start the OPAMPx calibration */
        let ref mut fresh12 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Stop the OPAMPx calibration */
        let ref mut fresh13 =
            *((0x40000000i32 as
                   uint32_t).wrapping_add(0x10000i32 as
                                              libc::c_uint).wrapping_add(0x38i32
                                                                             as
                                                                             libc::c_uint).wrapping_add(OPAMP_Selection)
                  as *mut uint32_t);
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  ******************************************************************************
  * @file    stm32f30x_opamp.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the operational
  *          amplifiers (OPAMP) firmware library.         
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
/* * @addtogroup OPAMP
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  OPAMP Init structure definition  
  */
/* !< Selects the inverting input of the operational amplifier.
                                          This parameter can be a value of @ref OPAMP_InvertingInput */
/* !< Selects the non inverting input of the operational amplifier.
                                         This parameter can be a value of @ref OPAMP_NonInvertingInput */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup OPAMP_Exported_Constants
  * @{
  */
/* * @defgroup OPAMP_Selection
  * @{
  */
/* !< OPAMP1 Selection */
/* !< OPAMP2 Selection */
/* !< OPAMP3 Selection */
/* !< OPAMP4 Selection */
/* *
  * @}
  */
/* * @defgroup OPAMP_InvertingInput
  * @{
  */
/* !< IO1 (PC5 for OPAMP1 and OPAMP2, PB10 for OPAMP3 and OPAMP4)
                                                                     connected to OPAMPx inverting input */
/* !< IO2 (PA3 for OPAMP1, PA5 for OPAMP2, PB2 for OPAMP3, PD8 for OPAMP4)
                                                                      connected to OPAMPx inverting input */
/* !< Resistor feedback output connected to OPAMPx inverting input (PGA mode) */
/* !< Vout connected to OPAMPx inverting input (follower mode) */
/* *
  * @}
  */
/* * @defgroup OPAMP_NonInvertingInput
  * @{
  */
/* !< IO1 (PA7 for OPAMP1, PD14 for OPAMP2, PB13 for OPAMP3, PD11 for OPAMP4)
                                                                        connected to OPAMPx non inverting input */
/* !< IO2 (PA5 for OPAMP1, PB14 for OPAMP2, PA5 for OPAMP3, PB11 for OPAMP4)
                                                                         connected to OPAMPx non inverting input */
/* !< IO3 (PA3 for OPAMP1, PB0 for OPAMP2, PA1 for OPAMP3, PA4 for OPAMP4)
                                                                         connected to OPAMPx non inverting input */
/* !< IO4 (PA1 for OPAMP1, PA7 for OPAMP2, PB0 for OPAMP3, PB13 for OPAMP4)
                                                                         connected to OPAMPx non inverting input */
/* *
  * @}
  */
/* * @defgroup OPAMP_PGAGain_Config
  * @{
  */
/* *
  * @}
  */
/* * @defgroup OPAMP_PGAConnect_Config
  * @{
  */
/* *
  * @}
  */
/* * @defgroup OPAMP_SecondaryInvertingInput
  * @{
  */
/* *
  * @}
  */
/* * @defgroup OPAMP_Input
  * @{
  */
/* !< Inverting input */
/* !< Non inverting input */
/* *
  * @}
  */
/* * @defgroup OPAMP_Vref
  * @{
  */
/* !< OPMAP Vref = 3.3% VDDA */
/* !< OPMAP Vref = 10% VDDA  */
/* !< OPMAP Vref = 50% VDDA  */
/* !< OPMAP Vref = 90% VDDA  */
/* *
  * @}
  */
/* * @defgroup OPAMP_Trimming
  */
/* !< Factory trimming */
/* !< User trimming */
/* *
  * @}
  */
/* * @defgroup OPAMP_TrimValue
  * @{
  */
/* !< Trimming value */
/* *
  * @}
  */
/* * @defgroup OPAMP_OutputLevel
  * @{
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*  Function used to set the OPAMP configuration to the default reset state ***/
/* Initialization and Configuration functions *********************************/
/* Calibration functions ******************************************************/
/* OPAMP configuration locking function ***************************************/
/* *
  * @}
  */
/* * @defgroup OPAMP_Group2 OPAMP configuration locking function
 *  @brief   OPAMP1,...OPAMP4 configuration locking function
 *           OPAMP1,...OPAMP4 configuration can be locked each separately.
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
  * @brief  Lock the selected OPAMP configuration.
  * @note   Locking the configuration means that all control bits are read-only.
  *         To unlock the OPAMP configuration, perform a system reset.
  * @param  OPAMP_Selection: the selected OPAMP. 
  *          This parameter can be OPAMP_Selection_OPAMPx where x can be 1 to 4
  *          to select the OPAMP peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn OPAMP_LockConfig(mut OPAMP_Selection: uint32_t) {
    /* Check the parameter */
    /* Set the lock bit corresponding to selected OPAMP */
    let ref mut fresh14 =
        *((0x40000000i32 as
               uint32_t).wrapping_add(0x10000i32 as
                                          libc::c_uint).wrapping_add(0x38i32
                                                                         as
                                                                         libc::c_uint).wrapping_add(OPAMP_Selection)
              as *mut uint32_t);
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x80000000u32) as
                                    uint32_t as uint32_t);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* *
  * @}
  */
