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
  * @brief Inter-integrated Circuit Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub OAR1: uint32_t,
    pub OAR2: uint32_t,
    pub TIMINGR: uint32_t,
    pub TIMEOUTR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub PECR: uint32_t,
    pub RXDR: uint32_t,
    pub TXDR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub I2C_Timing: uint32_t,
    pub I2C_AnalogFilter: uint32_t,
    pub I2C_DigitalFilter: uint32_t,
    pub I2C_Mode: uint32_t,
    pub I2C_OwnAddress1: uint32_t,
    pub I2C_Ack: uint32_t,
    pub I2C_AcknowledgedAddress: uint32_t,
}
/*<! I2C TC interrupt register Mask */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup I2C_Private_Functions
  * @{
  */
/* * @defgroup I2C_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim   
 ===============================================================================
           ##### Initialization and Configuration functions #####
 ===============================================================================
    [..] This section provides a set of functions allowing to initialize the I2C Mode,
         I2C Timing, I2C filters, I2C Addressing mode, I2C OwnAddress1.

    [..] The I2C_Init() function follows the I2C configuration procedures (these procedures 
         are available in reference manual).
         
    [..] When the Software Reset is performed using I2C_SoftwareResetCmd() function, the internal
         states machines are reset and communication control bits, as well as status bits come 
         back to their reset value.
         
    [..] Before enabling Stop mode using I2C_StopModeCmd() I2C Clock source must be set to
         HSI and Digital filters must be disabled.
         
    [..] Before enabling Own Address 2 via I2C_DualAddressCmd() function, OA2 and mask should be
         configured using I2C_OwnAddress2Config() function.
         
    [..] I2C_SlaveByteControlCmd() enable Slave byte control that allow user to get control of 
         each byte in slave mode when NBYTES is set to 0x01. 
             
@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the I2Cx peripheral registers to their default reset values.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_DeInit(mut I2Cx: *mut I2C_TypeDef) {
    /* Check the parameters */
    if I2Cx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x5400 as libc::c_int as libc::c_uint)
               as *mut I2C_TypeDef {
        /* Enable I2C1 reset state */
        RCC_APB1PeriphResetCmd(0x200000 as libc::c_int as uint32_t, ENABLE);
        /* Release I2C1 from reset state */
        RCC_APB1PeriphResetCmd(0x200000 as libc::c_int as uint32_t, DISABLE);
    } else {
        /* Enable I2C2 reset state */
        RCC_APB1PeriphResetCmd(0x400000 as libc::c_int as uint32_t, ENABLE);
        /* Release I2C2 from reset state */
        RCC_APB1PeriphResetCmd(0x400000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the I2Cx peripheral according to the specified
  *         parameters in the I2C_InitStruct.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that
  *         contains the configuration information for the specified I2C peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_Init(mut I2Cx: *mut I2C_TypeDef,
                                  mut I2C_InitStruct: *mut I2C_InitTypeDef) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Disable I2Cx Peripheral */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    /*---------------------------- I2Cx FILTERS Configuration ------------------*/
  /* Get the I2Cx CR1 value */
    tmpreg = (*I2Cx).CR1;
    /* Clear I2Cx CR1 register */
    tmpreg &= 0xcfe0ff as libc::c_int as uint32_t;
    /* Configure I2Cx: analog and digital filter */
  /* Set ANFOFF bit according to I2C_AnalogFilter value */
  /* Set DFN bits according to I2C_DigitalFilter value */
    tmpreg |=
        (*I2C_InitStruct).I2C_AnalogFilter |
            (*I2C_InitStruct).I2C_DigitalFilter << 8 as libc::c_int;
    /* Write to I2Cx CR1 */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t, tmpreg);
    /*---------------------------- I2Cx TIMING Configuration -------------------*/
  /* Configure I2Cx: Timing */
  /* Set TIMINGR bits according to I2C_Timing */
  /* Write to I2Cx TIMING */
    ::core::ptr::write_volatile(&mut (*I2Cx).TIMINGR as *mut uint32_t,
                                (*I2C_InitStruct).I2C_Timing &
                                    0xf0ffffff as libc::c_uint);
    /* Enable I2Cx Peripheral */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    /*---------------------------- I2Cx OAR1 Configuration ---------------------*/
  /* Clear tmpreg local variable */
    tmpreg = 0 as libc::c_int as uint32_t;
    /* Clear OAR1 register */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR1 as *mut uint32_t, tmpreg);
    /* Clear OAR2 register */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint32_t, tmpreg);
    /* Configure I2Cx: Own Address1 and acknowledged address */
  /* Set OA1MODE bit according to I2C_AcknowledgedAddress value */
  /* Set OA1 bits according to I2C_OwnAddress1 value */
    tmpreg =
        (*I2C_InitStruct).I2C_AcknowledgedAddress |
            (*I2C_InitStruct).I2C_OwnAddress1;
    /* Write to I2Cx OAR1 */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR1 as *mut uint32_t, tmpreg);
    /* Enable Own Address1 acknowledgement */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).OAR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x8000 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    /*---------------------------- I2Cx MODE Configuration ---------------------*/
  /* Configure I2Cx: mode */
  /* Set SMBDEN and SMBHEN bits according to I2C_Mode value */
    tmpreg = (*I2C_InitStruct).I2C_Mode;
    /* Write to I2Cx CR1 */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | tmpreg) as uint32_t as
                                    uint32_t);
    /*---------------------------- I2Cx ACK Configuration ----------------------*/
  /* Get the I2Cx CR2 value */
    tmpreg = (*I2Cx).CR2;
    /* Clear I2Cx CR2 register */
    tmpreg &= 0x7ff7fff as libc::c_int as uint32_t;
    /* Configure I2Cx: acknowledgement */
  /* Set NACK bit according to I2C_Ack value */
    tmpreg |= (*I2C_InitStruct).I2C_Ack;
    /* Write to I2Cx CR2 */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Fills each I2C_InitStruct member with its default value.
  * @param  I2C_InitStruct: pointer to an I2C_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_StructInit(mut I2C_InitStruct:
                                            *mut I2C_InitTypeDef) {
    /*---------------- Reset I2C init structure parameters values --------------*/
  /* Initialize the I2C_Timing member */
    (*I2C_InitStruct).I2C_Timing = 0 as libc::c_int as uint32_t;
    /* Initialize the I2C_AnalogFilter member */
    (*I2C_InitStruct).I2C_AnalogFilter = 0 as libc::c_int as uint32_t;
    /* Initialize the I2C_DigitalFilter member */
    (*I2C_InitStruct).I2C_DigitalFilter = 0 as libc::c_int as uint32_t;
    /* Initialize the I2C_Mode member */
    (*I2C_InitStruct).I2C_Mode = 0 as libc::c_int as uint32_t;
    /* Initialize the I2C_OwnAddress1 member */
    (*I2C_InitStruct).I2C_OwnAddress1 = 0 as libc::c_int as uint32_t;
    /* Initialize the I2C_Ack member */
    (*I2C_InitStruct).I2C_Ack = 0x8000 as libc::c_int as uint32_t;
    /* Initialize the I2C_AcknowledgedAddress member */
    (*I2C_InitStruct).I2C_AcknowledgedAddress = 0 as libc::c_int as uint32_t;
}
/* *
  * @brief  Enables or disables the specified I2C peripheral.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx peripheral. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_Cmd(mut I2Cx: *mut I2C_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C peripheral */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected I2C peripheral */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1 as libc::c_int as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C software reset.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SoftwareResetCmd(mut I2Cx: *mut I2C_TypeDef) {
    /* Check the parameters */
    /* Disable peripheral */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1 as libc::c_int as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Perform a dummy read to delay the disable of peripheral for minimum
     3 APB clock cycles to perform the software reset functionality */
    /* Enable peripheral */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the specified I2C interrupts.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_IT: specifies the I2C interrupts sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg I2C_IT_ERRI: Error interrupt mask
  *     @arg I2C_IT_TCI: Transfer Complete interrupt mask
  *     @arg I2C_IT_STOPI: Stop Detection interrupt mask
  *     @arg I2C_IT_NACKI: Not Acknowledge received interrupt mask
  *     @arg I2C_IT_ADDRI: Address Match interrupt mask  
  *     @arg I2C_IT_RXI: RX interrupt mask
  *     @arg I2C_IT_TXI: TX interrupt mask
  * @param  NewState: new state of the specified I2C interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ITConfig(mut I2Cx: *mut I2C_TypeDef,
                                      mut I2C_IT: uint32_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C interrupts */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | I2C_IT) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the selected I2C interrupts */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !I2C_IT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the I2C Clock stretching.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx Clock stretching.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_StretchClockCmd(mut I2Cx: *mut I2C_TypeDef,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable clock stretching */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x20000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable clock stretching  */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables I2C wakeup from stop mode.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx stop mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_StopModeCmd(mut I2Cx: *mut I2C_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable wakeup from stop mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable wakeup from stop mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x40000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the I2C own address 2.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C own address 2.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_DualAddressCmd(mut I2Cx: *mut I2C_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable own address 2 */
        ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).OAR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable own address 2 */
        ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).OAR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures the I2C slave own address 2 and mask.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Address: specifies the slave address to be programmed.
  * @param  Mask: specifies own address 2 mask to be programmed.
  *   This parameter can be one of the following values:
  *     @arg I2C_OA2_NoMask: no mask.
  *     @arg I2C_OA2_Mask01: OA2[1] is masked and don't care.
  *     @arg I2C_OA2_Mask02: OA2[2:1] are masked and don't care.
  *     @arg I2C_OA2_Mask03: OA2[3:1] are masked and don't care.
  *     @arg I2C_OA2_Mask04: OA2[4:1] are masked and don't care.
  *     @arg I2C_OA2_Mask05: OA2[5:1] are masked and don't care.
  *     @arg I2C_OA2_Mask06: OA2[6:1] are masked and don't care.
  *     @arg I2C_OA2_Mask07: OA2[7:1] are masked and don't care.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_OwnAddress2Config(mut I2Cx: *mut I2C_TypeDef,
                                               mut Address: uint16_t,
                                               mut Mask: uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*I2Cx).OAR2;
    /* Reset I2Cx OA2 bit [7:1] and OA2MSK bit [1:0]  */
    tmpreg &=
        !(0xfe as libc::c_int as uint32_t | 0x700 as libc::c_int as uint32_t);
    /* Set I2Cx SADD */
    tmpreg |=
        Address as uint32_t & 0xfe as libc::c_int as uint32_t |
            (Mask as uint32_t) << 8 as libc::c_int &
                0x700 as libc::c_int as uint32_t;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the I2C general call mode.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C general call mode.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GeneralCallCmd(mut I2Cx: *mut I2C_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable general call mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable general call mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the I2C slave byte control.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C slave byte control.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SlaveByteControlCmd(mut I2Cx: *mut I2C_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable slave byte control */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x10000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable slave byte control */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Configures the slave address to be transmitted after start generation.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Address: specifies the slave address to be programmed.
  * @note   This function should be called before generating start condition.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SlaveAddressConfig(mut I2Cx: *mut I2C_TypeDef,
                                                mut Address: uint16_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*I2Cx).CR2;
    /* Reset I2Cx SADD bit [9:0] */
    tmpreg &= !(0x3ff as libc::c_int as uint32_t);
    /* Set I2Cx SADD */
    tmpreg |= Address as uint32_t & 0x3ff as libc::c_int as uint32_t;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the I2C 10-bit addressing mode for the master.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C 10-bit addressing mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function should be called before generating start condition.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_10BitAddressingModeCmd(mut I2Cx:
                                                        *mut I2C_TypeDef,
                                                    mut NewState:
                                                        FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable 10-bit addressing mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable 10-bit addressing mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup I2C_Group2 Communications handling functions
 *  @brief   Communications handling functions 
 *
@verbatim
 ===============================================================================
                  ##### Communications handling functions #####
 ===============================================================================  
    [..] This section provides a set of functions that handles I2C communication.
    
    [..] Automatic End mode is enabled using I2C_AutoEndCmd() function. When Reload
         mode is enabled via I2C_ReloadCmd() AutoEnd bit has no effect.
         
    [..] I2C_NumberOfBytesConfig() function set the number of bytes to be transferred,
         this configuration should be done before generating start condition in master 
         mode.
         
    [..] When switching from master write operation to read operation in 10Bit addressing
         mode, master can only sends the 1st 7 bits of the 10 bit address, followed by 
         Read direction by enabling HEADR bit using I2C_10BitAddressHeader() function.        
         
    [..] In master mode, when transferring more than 255 bytes Reload mode should be used
         to handle communication. In the first phase of transfer, Nbytes should be set to 
         255. After transferring these bytes TCR flag is set and I2C_TransferHandling()
         function should be called to handle remaining communication.
         
    [..] In master mode, when software end mode is selected when all data is transferred
         TC flag is set I2C_TransferHandling() function should be called to generate STOP
         or generate ReStart.                      
             
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the I2C automatic end mode (stop condition is 
  *         automatically sent when nbytes data are transferred).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C automatic end mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This function has effect if Reload mode is disabled.   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_AutoEndCmd(mut I2Cx: *mut I2C_TypeDef,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable Auto end mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2000000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable Auto end mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2000000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables the I2C nbytes reload mode.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the nbytes reload mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ReloadCmd(mut I2Cx: *mut I2C_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable Auto Reload mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable Auto Reload mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Configures the number of bytes to be transmitted/received.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Number_Bytes: specifies the number of bytes to be programmed.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_NumberOfBytesConfig(mut I2Cx: *mut I2C_TypeDef,
                                                 mut Number_Bytes: uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*I2Cx).CR2;
    /* Reset I2Cx Nbytes bit [7:0] */
    tmpreg &= !(0xff0000 as libc::c_int as uint32_t);
    /* Set I2Cx Nbytes */
    tmpreg |=
        (Number_Bytes as uint32_t) << 16 as libc::c_int &
            0xff0000 as libc::c_int as uint32_t;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the type of transfer request for the master.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_Direction: specifies the transfer request direction to be programmed.
  *    This parameter can be one of the following values:
  *     @arg I2C_Direction_Transmitter: Master request a write transfer
  *     @arg I2C_Direction_Receiver: Master request a read transfer 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_MasterRequestConfig(mut I2Cx: *mut I2C_TypeDef,
                                                 mut I2C_Direction:
                                                     uint16_t) {
    /* Check the parameters */
    /* Test on the direction to set/reset the read/write bit */
    if I2C_Direction as libc::c_int ==
           0 as libc::c_int as uint16_t as libc::c_int {
        /* Request a write Transfer */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x400 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    } else {
        /* Request a read Transfer */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x400 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Generates I2Cx communication START condition.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C START condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GenerateSTART(mut I2Cx: *mut I2C_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Generate a START condition */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x2000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the START condition generation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x2000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Generates I2Cx communication STOP condition.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C STOP condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GenerateSTOP(mut I2Cx: *mut I2C_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Generate a STOP condition */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the STOP condition generation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the I2C 10-bit header only mode with read direction.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C 10-bit header only mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @note   This mode can be used only when switching from master transmitter mode 
  *         to master receiver mode.        
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_10BitAddressHeaderCmd(mut I2Cx: *mut I2C_TypeDef,
                                                   mut NewState:
                                                       FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable 10-bit header only mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable 10-bit header only mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Generates I2C communication Acknowledge.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the Acknowledge.
  *   This parameter can be: ENABLE or DISABLE.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_AcknowledgeConfig(mut I2Cx: *mut I2C_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable ACK generation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    } else {
        /* Enable NACK generation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Returns the I2C slave matched address .
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval The value of the slave matched address .
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetAddressMatched(mut I2Cx: *mut I2C_TypeDef)
 -> uint8_t {
    /* Check the parameters */
    /* Return the slave matched address in the SR1 register */
    return (((*I2Cx).ISR & 0xfe0000 as libc::c_int as uint32_t) >>
                16 as libc::c_int) as uint8_t;
}
/* *
  * @brief  Returns the I2C slave received request.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval The value of the received request.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetTransferDirection(mut I2Cx: *mut I2C_TypeDef)
 -> uint16_t {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut direction: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Return the slave matched address in the SR1 register */
    tmpreg = (*I2Cx).ISR & 0x10000 as libc::c_int as uint32_t;
    /* If write transfer is requested */
    if tmpreg == 0 as libc::c_int as libc::c_uint {
        /* write transfer is requested */
        direction = 0 as libc::c_int as uint16_t
    } else {
        /* Read transfer is requested */
        direction = 0x400 as libc::c_int as uint16_t
    }
    return direction;
}
/* *
  * @brief  Handles I2Cx communication when starting transfer or during transfer (TC or TCR flag are set).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Address: specifies the slave address to be programmed.
  * @param  Number_Bytes: specifies the number of bytes to be programmed.
  *   This parameter must be a value between 0 and 255.
  * @param  ReloadEndMode: new state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg I2C_Reload_Mode: Enable Reload mode .
  *     @arg I2C_AutoEnd_Mode: Enable Automatic end mode.
  *     @arg I2C_SoftEnd_Mode: Enable Software end mode.
  * @param  StartStopMode: new state of the I2C START condition generation.
  *   This parameter can be one of the following values:
  *     @arg I2C_No_StartStop: Don't Generate stop and start condition.
  *     @arg I2C_Generate_Stop: Generate stop condition (Number_Bytes should be set to 0).
  *     @arg I2C_Generate_Start_Read: Generate Restart for read request.
  *     @arg I2C_Generate_Start_Write: Generate Restart for write request.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_TransferHandling(mut I2Cx: *mut I2C_TypeDef,
                                              mut Address: uint16_t,
                                              mut Number_Bytes: uint8_t,
                                              mut ReloadEndMode: uint32_t,
                                              mut StartStopMode: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the CR2 register value */
    tmpreg = (*I2Cx).CR2;
    /* clear tmpreg specific bits */
    tmpreg &=
        !(0x3ff as libc::c_int as uint32_t |
              0xff0000 as libc::c_int as uint32_t |
              0x1000000 as libc::c_int as uint32_t |
              0x2000000 as libc::c_int as uint32_t |
              0x400 as libc::c_int as uint32_t |
              0x2000 as libc::c_int as uint32_t |
              0x4000 as libc::c_int as uint32_t);
    /* update tmpreg */
    tmpreg |=
        Address as uint32_t & 0x3ff as libc::c_int as uint32_t |
            (Number_Bytes as uint32_t) << 16 as libc::c_int &
                0xff0000 as libc::c_int as uint32_t | ReloadEndMode |
            StartStopMode;
    /* update CR2 register */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint32_t, tmpreg);
}
/* *
  * @}
  */
/* * @defgroup I2C_Group3 SMBUS management functions
 *  @brief   SMBUS management functions 
 *
@verbatim
 ===============================================================================
                      ##### SMBUS management functions #####
 ===============================================================================   
    [..] This section provides a set of functions that handles SMBus communication
         and timeouts detection.
    
    [..] The SMBus Device default address (0b1100 001) is enabled by calling I2C_Init()
         function and setting I2C_Mode member of I2C_InitTypeDef() structure to 
         I2C_Mode_SMBusDevice.
         
    [..] The SMBus Host address (0b0001 000) is enabled by calling I2C_Init()
         function and setting I2C_Mode member of I2C_InitTypeDef() structure to 
         I2C_Mode_SMBusHost.         
         
    [..] The Alert Response Address (0b0001 100) is enabled using I2C_SMBusAlertCmd()
         function.
         
    [..] To detect cumulative SCL stretch in master and slave mode, TIMEOUTB should be 
         configured (in accordance to SMBus specification) using I2C_TimeoutBConfig() 
         function then I2C_ExtendedClockTimeoutCmd() function should be called to enable
         the detection.
         
    [..] SCL low timeout is detected by configuring TIMEOUTB using I2C_TimeoutBConfig()
         function followed by the call of I2C_ClockTimeoutCmd(). When adding to this 
         procedure the call of I2C_IdleClockTimeoutCmd() function, Bus Idle condition 
         (both SCL and SDA high) is detected also.                
                          
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables I2C SMBus alert.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx SMBus alert.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SMBusAlertCmd(mut I2Cx: *mut I2C_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable SMBus alert */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x400000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable SMBus alert */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x400000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables I2C Clock Timeout (SCL Timeout detection).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx clock Timeout.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ClockTimeoutCmd(mut I2Cx: *mut I2C_TypeDef,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable Clock Timeout */
        ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).TIMEOUTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable Clock Timeout */
        ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).TIMEOUTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables I2C Extended Clock Timeout (SCL cumulative Timeout detection).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx Extended clock Timeout.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ExtendedClockTimeoutCmd(mut I2Cx:
                                                         *mut I2C_TypeDef,
                                                     mut NewState:
                                                         FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable Clock Timeout */
        ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).TIMEOUTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80000000 as libc::c_uint) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable Clock Timeout */
        ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).TIMEOUTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80000000 as libc::c_uint)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables I2C Idle Clock Timeout (Bus idle SCL and SDA 
  *         high detection).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx Idle clock Timeout.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_IdleClockTimeoutCmd(mut I2Cx: *mut I2C_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable Clock Timeout */
        ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).TIMEOUTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000 as libc::c_int as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable Clock Timeout */
        ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).TIMEOUTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000 as libc::c_int as uint32_t))
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures the I2C Bus Timeout A (SCL Timeout when TIDLE = 0 or Bus 
  *   idle SCL and SDA high when TIDLE = 1).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Timeout: specifies the TimeoutA to be programmed. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_TimeoutAConfig(mut I2Cx: *mut I2C_TypeDef,
                                            mut Timeout: uint16_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*I2Cx).TIMEOUTR;
    /* Reset I2Cx TIMEOUTA bit [11:0] */
    tmpreg &= !(0xfff as libc::c_int as uint32_t);
    /* Set I2Cx TIMEOUTA */
    tmpreg |= Timeout as uint32_t & 0xfff as libc::c_int as uint32_t;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                tmpreg);
}
/* *
  * @brief  Configures the I2C Bus Timeout B (SCL cumulative Timeout).
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Timeout: specifies the TimeoutB to be programmed. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_TimeoutBConfig(mut I2Cx: *mut I2C_TypeDef,
                                            mut Timeout: uint16_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*I2Cx).TIMEOUTR;
    /* Reset I2Cx TIMEOUTB bit [11:0] */
    tmpreg &= !(0xfff0000 as libc::c_int as uint32_t);
    /* Set I2Cx TIMEOUTB */
    tmpreg |=
        (Timeout as uint32_t) << 16 as libc::c_int &
            0xfff0000 as libc::c_int as uint32_t;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*I2Cx).TIMEOUTR as *mut uint32_t,
                                tmpreg);
}
/* *
  * @brief  Enables or disables I2C PEC calculation.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx PEC calculation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_CalculatePEC(mut I2Cx: *mut I2C_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable PEC calculation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable PEC calculation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Enables or disables I2C PEC transmission/reception request.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx PEC request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_PECRequestCmd(mut I2Cx: *mut I2C_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable PEC transmission/reception request */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x4000000 as libc::c_int as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Disable PEC transmission/reception request */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4000000 as libc::c_int as
                                               uint32_t)) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Returns the I2C PEC.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval The value of the PEC .
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetPEC(mut I2Cx: *mut I2C_TypeDef) -> uint8_t {
    /* Check the parameters */
    /* Return the slave matched address in the SR1 register */
    return ((*I2Cx).PECR & 0xff as libc::c_int as uint32_t) as uint8_t;
}
/* *
  * @}
  */
/* * @defgroup I2C_Group4 I2C registers management functions
 *  @brief   I2C registers management functions 
 *
@verbatim
 ===============================================================================
                ##### I2C registers management functions #####
 ===============================================================================  
    [..] This section provides a functions that allow user the management of 
         I2C registers.
         
@endverbatim
  * @{
  */
/* *
  * @brief  Reads the specified I2C register and returns its value.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_Register: specifies the register to read.
  *   This parameter can be one of the following values:
  *     @arg I2C_Register_CR1: CR1 register.
  *     @arg I2C_Register_CR2: CR2 register.
  *     @arg I2C_Register_OAR1: OAR1 register.
  *     @arg I2C_Register_OAR2: OAR2 register.
  *     @arg I2C_Register_TIMINGR: TIMING register.
  *     @arg I2C_Register_TIMEOUTR: TIMEOUTR register.
  *     @arg I2C_Register_ISR: ISR register.
  *     @arg I2C_Register_ICR: ICR register.
  *     @arg I2C_Register_PECR: PECR register.
  *     @arg I2C_Register_RXDR: RXDR register.
  *     @arg I2C_Register_TXDR: TXDR register.
  * @retval The value of the read register.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ReadRegister(mut I2Cx: *mut I2C_TypeDef,
                                          mut I2C_Register: uint8_t)
 -> uint32_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t, I2Cx as uint32_t);
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(I2C_Register
                                                                    as
                                                                    libc::c_uint)
                                    as uint32_t as uint32_t);
    /* Return the selected register value */
    return *(tmp as *mut uint32_t);
}
/* *
  * @}
  */
/* * @defgroup I2C_Group5 Data transfers management functions
 *  @brief   Data transfers management functions 
 *
@verbatim
 ===============================================================================
                ##### Data transfers management functions #####
 =============================================================================== 
    [..] This subsection provides a set of functions allowing to manage 
         the I2C data transfers.
         
    [..] The read access of the I2C_RXDR register can be done using 
         the I2C_ReceiveData() function and returns the received value.
         Whereas a write access to the I2C_TXDR can be done using I2C_SendData()
         function and stores the written data into TXDR.
@endverbatim
  * @{
  */
/* *
  * @brief  Sends a data byte through the I2Cx peripheral.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Data: Byte to be transmitted..
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SendData(mut I2Cx: *mut I2C_TypeDef,
                                      mut Data: uint8_t) {
    /* Check the parameters */
    /* Write in the DR register the data to be sent */
    ::core::ptr::write_volatile(&mut (*I2Cx).TXDR as *mut uint32_t,
                                Data as uint32_t);
}
/* *
  * @brief  Returns the most recent received data by the I2Cx peripheral.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval The value of the received data.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ReceiveData(mut I2Cx: *mut I2C_TypeDef)
 -> uint8_t {
    /* Check the parameters */
    /* Return the data in the DR register */
    return (*I2Cx).RXDR as uint8_t;
}
/* *
  * @}
  */
/* * @defgroup I2C_Group6 DMA transfers management functions
 *  @brief   DMA transfers management functions 
 *
@verbatim
 ===============================================================================
               ##### DMA transfers management functions #####
 ===============================================================================  
    [..] This section provides two functions that can be used only in DMA mode.
    [..] In DMA Mode, the I2C communication can be managed by 2 DMA Channel 
         requests:
         (#) I2C_DMAReq_Tx: specifies the Tx buffer DMA transfer request.
         (#) I2C_DMAReq_Rx: specifies the Rx buffer DMA transfer request.
    [..] In this Mode it is advised to use the following function:
         (+) I2C_DMACmd(I2C_TypeDef* I2Cx, uint32_t I2C_DMAReq, FunctionalState NewState);
@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the I2C DMA interface.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_DMAReq: specifies the I2C DMA transfer request to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg I2C_DMAReq_Tx: Tx DMA transfer request
  *     @arg I2C_DMAReq_Rx: Rx DMA transfer request
  * @param  NewState: new state of the selected I2C DMA transfer request.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_DMACmd(mut I2Cx: *mut I2C_TypeDef,
                                    mut I2C_DMAReq: uint32_t,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C DMA requests */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | I2C_DMAReq) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected I2C DMA requests */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !I2C_DMAReq) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup I2C_Group7 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions 
 *
@verbatim
 ===============================================================================
             ##### Interrupts and flags management functions  #####
 =============================================================================== 
    [..] This section provides functions allowing to configure the I2C Interrupts 
         sources and check or clear the flags or pending bits status.
         The user should identify which mode will be used in his application to manage 
         the communication: Polling mode, Interrupt mode or DMA mode(refer I2C_Group6) .

  *** Polling Mode ***
  ====================
    [..] In Polling Mode, the I2C communication can be managed by 15 flags:
        (#) I2C_FLAG_TXE: to indicate the status of Transmit data register empty flag.
        (#) I2C_FLAG_TXIS: to indicate the status of Transmit interrupt status flag .
        (#) I2C_FLAG_RXNE: to indicate the status of Receive data register not empty flag.
        (#) I2C_FLAG_ADDR: to indicate the status of Address matched flag (slave mode).
        (#) I2C_FLAG_NACKF: to indicate the status of NACK received flag.
        (#) I2C_FLAG_STOPF: to indicate the status of STOP detection flag.
        (#) I2C_FLAG_TC: to indicate the status of Transfer complete flag(master mode).
        (#) I2C_FLAG_TCR: to indicate the status of Transfer complete reload flag.
        (#) I2C_FLAG_BERR: to indicate the status of Bus error flag.
        (#) I2C_FLAG_ARLO: to indicate the status of Arbitration lost flag.
        (#) I2C_FLAG_OVR: to indicate the status of Overrun/Underrun flag.
        (#) I2C_FLAG_PECERR: to indicate the status of PEC error in reception flag.
        (#) I2C_FLAG_TIMEOUT: to indicate the status of Timeout or Tlow detection flag.
        (#) I2C_FLAG_ALERT: to indicate the status of SMBus Alert flag.
        (#) I2C_FLAG_BUSY: to indicate the status of Bus busy flag.

    [..] In this Mode it is advised to use the following functions:
        (+) FlagStatus I2C_GetFlagStatus(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);
        (+) void I2C_ClearFlag(I2C_TypeDef* I2Cx, uint32_t I2C_FLAG);

    [..]
        (@)Do not use the BUSY flag to handle each data transmission or reception.It is 
           better to use the TXIS and RXNE flags instead.

  *** Interrupt Mode ***
  ======================
    [..] In Interrupt Mode, the I2C communication can be managed by 7 interrupt sources
         and 15 pending bits: 
    [..] Interrupt Source:
        (#) I2C_IT_ERRI: specifies the interrupt source for the Error interrupt.
        (#) I2C_IT_TCI: specifies the interrupt source for the Transfer Complete interrupt.
        (#) I2C_IT_STOPI: specifies the interrupt source for the Stop Detection interrupt.
        (#) I2C_IT_NACKI: specifies the interrupt source for the Not Acknowledge received interrupt.
        (#) I2C_IT_ADDRI: specifies the interrupt source for the Address Match interrupt.  
        (#) I2C_IT_RXI: specifies the interrupt source for the RX interrupt.
        (#) I2C_IT_TXI: specifies the interrupt source for the TX interrupt.

    [..] Pending Bits:
        (#) I2C_IT_TXIS: to indicate the status of Transmit interrupt status flag.
        (#) I2C_IT_RXNE: to indicate the status of Receive data register not empty flag.
        (#) I2C_IT_ADDR: to indicate the status of Address matched flag (slave mode).
        (#) I2C_IT_NACKF: to indicate the status of NACK received flag.
        (#) I2C_IT_STOPF: to indicate the status of STOP detection flag.
        (#) I2C_IT_TC: to indicate the status of Transfer complete flag (master mode).
        (#) I2C_IT_TCR: to indicate the status of Transfer complete reload flag.
        (#) I2C_IT_BERR: to indicate the status of Bus error flag.
        (#) I2C_IT_ARLO: to indicate the status of Arbitration lost flag.
        (#) I2C_IT_OVR: to indicate the status of Overrun/Underrun flag.
        (#) I2C_IT_PECERR: to indicate the status of PEC error in reception flag.
        (#) I2C_IT_TIMEOUT: to indicate the status of Timeout or Tlow detection flag.
        (#) I2C_IT_ALERT: to indicate the status of SMBus Alert flag.

    [..] In this Mode it is advised to use the following functions:
         (+) void I2C_ClearITPendingBit(I2C_TypeDef* I2Cx, uint32_t I2C_IT);
         (+) ITStatus I2C_GetITStatus(I2C_TypeDef* I2Cx, uint32_t I2C_IT);

@endverbatim
  * @{
  */
/* *
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg I2C_FLAG_TXE: Transmit data register empty
  *     @arg I2C_FLAG_TXIS: Transmit interrupt status
  *     @arg I2C_FLAG_RXNE: Receive data register not empty
  *     @arg I2C_FLAG_ADDR: Address matched (slave mode)
  *     @arg I2C_FLAG_NACKF: NACK received flag
  *     @arg I2C_FLAG_STOPF: STOP detection flag
  *     @arg I2C_FLAG_TC: Transfer complete (master mode)
  *     @arg I2C_FLAG_TCR: Transfer complete reload
  *     @arg I2C_FLAG_BERR: Bus error
  *     @arg I2C_FLAG_ARLO: Arbitration lost
  *     @arg I2C_FLAG_OVR: Overrun/Underrun
  *     @arg I2C_FLAG_PECERR: PEC error in reception
  *     @arg I2C_FLAG_TIMEOUT: Timeout or Tlow detection flag
  *     @arg I2C_FLAG_ALERT: SMBus Alert
  *     @arg I2C_FLAG_BUSY: Bus busy
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetFlagStatus(mut I2Cx: *mut I2C_TypeDef,
                                           mut I2C_FLAG: uint32_t)
 -> FlagStatus {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Get the ISR register value */
    tmpreg = (*I2Cx).ISR;
    /* Get flag status */
    tmpreg &= I2C_FLAG;
    if tmpreg != 0 as libc::c_int as libc::c_uint {
        /* I2C_FLAG is set */
        bitstatus = SET
    } else {
        /* I2C_FLAG is reset */
        bitstatus = RESET
    }
    return bitstatus;
}
/* *
  * @brief  Clears the I2Cx's pending flags.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to clear. 
  *   This parameter can be any combination of the following values:
  *     @arg I2C_FLAG_ADDR: Address matched (slave mode)
  *     @arg I2C_FLAG_NACKF: NACK received flag
  *     @arg I2C_FLAG_STOPF: STOP detection flag
  *     @arg I2C_FLAG_BERR: Bus error
  *     @arg I2C_FLAG_ARLO: Arbitration lost
  *     @arg I2C_FLAG_OVR: Overrun/Underrun
  *     @arg I2C_FLAG_PECERR: PEC error in reception
  *     @arg I2C_FLAG_TIMEOUT: Timeout or Tlow detection flag
  *     @arg I2C_FLAG_ALERT: SMBus Alert
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ClearFlag(mut I2Cx: *mut I2C_TypeDef,
                                       mut I2C_FLAG: uint32_t) {
    /* Check the parameters */
    /* Clear the selected flag */
    ::core::ptr::write_volatile(&mut (*I2Cx).ICR as *mut uint32_t, I2C_FLAG);
}
/* *
  * @brief  Checks whether the specified I2C interrupt has occurred or not.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg I2C_IT_TXIS: Transmit interrupt status
  *     @arg I2C_IT_RXNE: Receive data register not empty
  *     @arg I2C_IT_ADDR: Address matched (slave mode)
  *     @arg I2C_IT_NACKF: NACK received flag
  *     @arg I2C_IT_STOPF: STOP detection flag
  *     @arg I2C_IT_TC: Transfer complete (master mode)
  *     @arg I2C_IT_TCR: Transfer complete reload
  *     @arg I2C_IT_BERR: Bus error
  *     @arg I2C_IT_ARLO: Arbitration lost
  *     @arg I2C_IT_OVR: Overrun/Underrun
  *     @arg I2C_IT_PECERR: PEC error in reception
  *     @arg I2C_IT_TIMEOUT: Timeout or Tlow detection flag
  *     @arg I2C_IT_ALERT: SMBus Alert
  * @retval The new state of I2C_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetITStatus(mut I2Cx: *mut I2C_TypeDef,
                                         mut I2C_IT: uint32_t) -> ITStatus {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut bitstatus: ITStatus = RESET;
    let mut enablestatus: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Check if the interrupt source is enabled or not */
  /* If Error interrupt */
    if I2C_IT & 0x3f00 as libc::c_int as uint32_t != 0 {
        enablestatus = 0x80 as libc::c_int as uint32_t & (*I2Cx).CR1
    } else if I2C_IT & 0xc0 as libc::c_int as uint32_t != 0 {
        enablestatus = 0x40 as libc::c_int as uint32_t & (*I2Cx).CR1
    } else { enablestatus = I2C_IT & (*I2Cx).CR1 }
    /* If TC interrupt */
    /* Get the ISR register value */
    tmpreg = (*I2Cx).ISR;
    /* Get flag status */
    tmpreg &= I2C_IT;
    /* Check the status of the specified I2C flag */
    if tmpreg != RESET as libc::c_int as libc::c_uint && enablestatus != 0 {
        /* I2C_IT is set */
        bitstatus = SET
    } else {
        /* I2C_IT is reset */
        bitstatus = RESET
    }
    /* Return the I2C_IT status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_i2c.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the I2C firmware
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
/* * @addtogroup I2C
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* *
  * @brief  I2C Init structure definition
  */
/* !< Specifies the I2C_TIMINGR_register value.
                                         This parameter calculated by referring to I2C initialization 
                                         section in Reference manual*/
/* !< Enables or disables analog noise filter.
                                         This parameter can be a value of @ref I2C_Analog_Filter */
/* !< Configures the digital noise filter.
                                         This parameter can be a number between 0x00 and 0x0F */
/* !< Specifies the I2C mode.
                                         This parameter can be a value of @ref I2C_mode */
/* !< Specifies the device own address 1.
                                         This parameter can be a 7-bit or 10-bit address */
/* !< Enables or disables the acknowledgement.
                                         This parameter can be a value of @ref I2C_acknowledgement */
/* !< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref I2C_acknowledged_address */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup I2C_Exported_Constants
  * @{
  */
/* * @defgroup I2C_Analog_Filter 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_Digital_Filter
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_acknowledgement
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_acknowledged_address
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_own_address1
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_transfer_direction 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_DMA_transfer_requests 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_slave_address
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_own_address2
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_own_address2_mask
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_timeout
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_registers 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_ReloadEndMode_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_StartStopMode_definition 
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
/* Communications handling functions ******************************************/
/*  SMBUS management functions ************************************************/
/* I2C registers management functions *****************************************/
/* Data transfers management functions ****************************************/
/* DMA transfers management functions *****************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the I2Cx's interrupt pending bits.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg I2C_IT_ADDR: Address matched (slave mode)
  *     @arg I2C_IT_NACKF: NACK received flag
  *     @arg I2C_IT_STOPF: STOP detection flag
  *     @arg I2C_IT_BERR: Bus error
  *     @arg I2C_IT_ARLO: Arbitration lost
  *     @arg I2C_IT_OVR: Overrun/Underrun
  *     @arg I2C_IT_PECERR: PEC error in reception
  *     @arg I2C_IT_TIMEOUT: Timeout or Tlow detection flag
  *     @arg I2C_IT_ALERT: SMBus Alert
  * @retval The new state of I2C_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ClearITPendingBit(mut I2Cx: *mut I2C_TypeDef,
                                               mut I2C_IT: uint32_t) {
    /* Check the parameters */
    /* Clear the selected flag */
    ::core::ptr::write_volatile(&mut (*I2Cx).ICR as *mut uint32_t, I2C_IT);
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
