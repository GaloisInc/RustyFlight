use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_GetClocksFreq(RCC_Clocks: *mut RCC_ClocksTypeDef);
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
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
/* * 
  * @brief Inter Integrated Circuit Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub OAR1: uint16_t,
    pub RESERVED2: uint16_t,
    pub OAR2: uint16_t,
    pub RESERVED3: uint16_t,
    pub DR: uint16_t,
    pub RESERVED4: uint16_t,
    pub SR1: uint16_t,
    pub RESERVED5: uint16_t,
    pub SR2: uint16_t,
    pub RESERVED6: uint16_t,
    pub CCR: uint16_t,
    pub RESERVED7: uint16_t,
    pub TRISE: uint16_t,
    pub RESERVED8: uint16_t,
}
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
pub struct I2C_InitTypeDef {
    pub I2C_ClockSpeed: uint32_t,
    pub I2C_Mode: uint16_t,
    pub I2C_DutyCycle: uint16_t,
    pub I2C_OwnAddress1: uint16_t,
    pub I2C_Ack: uint16_t,
    pub I2C_AcknowledgedAddress: uint16_t,
}
/* *
  * @}
  */
/* * @defgroup I2C_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_Private_Functions
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
  *   parameters in the I2C_InitStruct.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_InitStruct: pointer to a I2C_InitTypeDef structure that
  *   contains the configuration information for the specified I2C peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_Init(mut I2Cx: *mut I2C_TypeDef,
                                  mut I2C_InitStruct: *mut I2C_InitTypeDef) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    let mut freqrange: uint16_t = 0 as libc::c_int as uint16_t;
    let mut result: uint16_t = 0x4 as libc::c_int as uint16_t;
    let mut pclk1: uint32_t = 8000000 as libc::c_int as uint32_t;
    let mut rcc_clocks: RCC_ClocksTypeDef =
        RCC_ClocksTypeDef{SYSCLK_Frequency: 0,
                          HCLK_Frequency: 0,
                          PCLK1_Frequency: 0,
                          PCLK2_Frequency: 0,
                          ADCCLK_Frequency: 0,};
    /* Check the parameters */
    /*---------------------------- I2Cx CR2 Configuration ------------------------*/
  /* Get the I2Cx CR2 value */
    tmpreg = (*I2Cx).CR2;
    /* Clear frequency FREQ[5:0] bits */
    tmpreg =
        (tmpreg as libc::c_int &
             0xffc0 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    /* Get pclk1 frequency value */
    RCC_GetClocksFreq(&mut rcc_clocks);
    pclk1 = rcc_clocks.PCLK1_Frequency;
    /* Set frequency bits depending on pclk1 value */
    freqrange =
        pclk1.wrapping_div(1000000 as libc::c_int as libc::c_uint) as
            uint16_t;
    tmpreg = (tmpreg as libc::c_int | freqrange as libc::c_int) as uint16_t;
    /* Write to I2Cx CR2 */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t, tmpreg);
    /*---------------------------- I2Cx CCR Configuration ------------------------*/
  /* Disable the selected I2C peripheral to configure TRISE */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xfffe as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Reset tmpreg value */
  /* Clear F/S, DUTY and CCR[11:0] bits */
    tmpreg = 0 as libc::c_int as uint16_t;
    /* Configure speed in standard mode */
    if (*I2C_InitStruct).I2C_ClockSpeed <=
           100000 as libc::c_int as libc::c_uint {
        /* Standard mode speed calculate */
        result =
            pclk1.wrapping_div((*I2C_InitStruct).I2C_ClockSpeed <<
                                   1 as libc::c_int) as uint16_t;
        /* Test if CCR value is under 0x4*/
        if (result as libc::c_int) < 0x4 as libc::c_int {
            /* Set minimum allowed value */
            result = 0x4 as libc::c_int as uint16_t
        }
        /* Set speed value for standard mode */
        tmpreg = (tmpreg as libc::c_int | result as libc::c_int) as uint16_t;
        /* Set Maximum Rise Time for standard mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).TRISE as *mut uint16_t,
                                    (freqrange as libc::c_int +
                                         1 as libc::c_int) as uint16_t)
    } else {
        /* Configure speed in fast mode */
        /*(I2C_InitStruct->I2C_ClockSpeed <= 400000)*/
        if (*I2C_InitStruct).I2C_DutyCycle as libc::c_int ==
               0xbfff as libc::c_int as uint16_t as libc::c_int {
            /* Fast mode speed calculate: Tlow/Thigh = 2 */
            result =
                pclk1.wrapping_div((*I2C_InitStruct).I2C_ClockSpeed.wrapping_mul(3
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint))
                    as uint16_t
        } else {
            /*I2C_InitStruct->I2C_DutyCycle == I2C_DutyCycle_16_9*/
            /* Fast mode speed calculate: Tlow/Thigh = 16/9 */
            result =
                pclk1.wrapping_div((*I2C_InitStruct).I2C_ClockSpeed.wrapping_mul(25
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     libc::c_uint))
                    as uint16_t;
            result =
                (result as libc::c_int |
                     0x4000 as libc::c_int as uint16_t as libc::c_int) as
                    uint16_t
        }
        /* Set DUTY bit */
        /* Test if CCR value is under 0x1*/
        if result as libc::c_int &
               0xfff as libc::c_int as uint16_t as libc::c_int ==
               0 as libc::c_int {
            /* Set minimum allowed value */
            result =
                (result as libc::c_int |
                     0x1 as libc::c_int as uint16_t as libc::c_int) as
                    uint16_t
        }
        /* Set speed value and set F/S bit for fast mode */
        tmpreg =
            (tmpreg as libc::c_int |
                 (result as libc::c_int |
                      0x8000 as libc::c_int as uint16_t as libc::c_int) as
                     uint16_t as libc::c_int) as uint16_t;
        /* Set Maximum Rise Time for fast mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).TRISE as *mut uint16_t,
                                    (freqrange as libc::c_int *
                                         300 as libc::c_int as uint16_t as
                                             libc::c_int /
                                         1000 as libc::c_int as uint16_t as
                                             libc::c_int +
                                         1 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t)
    }
    /* Write to I2Cx CCR */
    ::core::ptr::write_volatile(&mut (*I2Cx).CCR as *mut uint16_t, tmpreg);
    /* Enable the selected I2C peripheral */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x1 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /*---------------------------- I2Cx CR1 Configuration ------------------------*/
  /* Get the I2Cx CR1 value */
    tmpreg = (*I2Cx).CR1;
    /* Clear ACK, SMBTYPE and  SMBUS bits */
    tmpreg =
        (tmpreg as libc::c_int &
             0xfbf5 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    /* Configure I2Cx: mode and acknowledgement */
  /* Set SMBTYPE and SMBUS bits according to I2C_Mode value */
  /* Set ACK bit according to I2C_Ack value */
    tmpreg =
        (tmpreg as libc::c_int |
             ((*I2C_InitStruct).I2C_Mode as uint32_t |
                  (*I2C_InitStruct).I2C_Ack as libc::c_uint) as uint16_t as
                 libc::c_int) as uint16_t;
    /* Write to I2Cx CR1 */
    ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t, tmpreg);
    /*---------------------------- I2Cx OAR1 Configuration -----------------------*/
  /* Set I2Cx Own Address1 and acknowledged address */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR1 as *mut uint16_t,
                                ((*I2C_InitStruct).I2C_AcknowledgedAddress as
                                     libc::c_int |
                                     (*I2C_InitStruct).I2C_OwnAddress1 as
                                         libc::c_int) as uint16_t);
}
/* *
  * @brief  Fills each I2C_InitStruct member with its default value.
  * @param  I2C_InitStruct: pointer to an I2C_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_StructInit(mut I2C_InitStruct:
                                            *mut I2C_InitTypeDef) {
    /*---------------- Reset I2C init structure parameters values ----------------*/
  /* initialize the I2C_ClockSpeed member */
    (*I2C_InitStruct).I2C_ClockSpeed = 5000 as libc::c_int as uint32_t;
    /* Initialize the I2C_Mode member */
    (*I2C_InitStruct).I2C_Mode = 0 as libc::c_int as uint16_t;
    /* Initialize the I2C_DutyCycle member */
    (*I2C_InitStruct).I2C_DutyCycle = 0xbfff as libc::c_int as uint16_t;
    /* Initialize the I2C_OwnAddress1 member */
    (*I2C_InitStruct).I2C_OwnAddress1 = 0 as libc::c_int as uint16_t;
    /* Initialize the I2C_Ack member */
    (*I2C_InitStruct).I2C_Ack = 0 as libc::c_int as uint16_t;
    /* Initialize the I2C_AcknowledgedAddress member */
    (*I2C_InitStruct).I2C_AcknowledgedAddress =
        0x4000 as libc::c_int as uint16_t;
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
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x1 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C peripheral */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfffe as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C DMA requests.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C DMA transfer.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_DMACmd(mut I2Cx: *mut I2C_TypeDef,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C DMA requests */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x800 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C DMA requests */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xf7ff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Specifies if the next DMA transfer will be the last one.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C DMA last transfer.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_DMALastTransferCmd(mut I2Cx: *mut I2C_TypeDef,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Next DMA transfer is the last transfer */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x1000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Next DMA transfer is not the last transfer */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xefff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Generates I2Cx communication START condition.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C START condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GenerateSTART(mut I2Cx: *mut I2C_TypeDef,
                                           mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Generate a START condition */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x100 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the START condition generation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfeff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Generates I2Cx communication STOP condition.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C STOP condition generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GenerateSTOP(mut I2Cx: *mut I2C_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Generate a STOP condition */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x200 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the STOP condition generation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfdff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C acknowledge feature.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C Acknowledgement.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_AcknowledgeConfig(mut I2Cx: *mut I2C_TypeDef,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the acknowledgement */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x400 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the acknowledgement */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfbff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Configures the specified I2C own address2.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Address: specifies the 7bit I2C own address2.
  * @retval None.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_OwnAddress2Config(mut I2Cx: *mut I2C_TypeDef,
                                               mut Address: uint8_t) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Get the old register value */
    tmpreg = (*I2Cx).OAR2;
    /* Reset I2Cx Own address2 bit [7:1] */
    tmpreg =
        (tmpreg as libc::c_int &
             0xff01 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    /* Set I2Cx Own address2 */
    tmpreg =
        (tmpreg as libc::c_int |
             (Address as uint16_t as libc::c_int &
                  0xfe as libc::c_int as uint16_t as libc::c_int) as uint16_t
                 as libc::c_int) as uint16_t;
    /* Store the new register value */
    ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint16_t, tmpreg);
}
/* *
  * @brief  Enables or disables the specified I2C dual addressing mode.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C dual addressing mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_DualAddressCmd(mut I2Cx: *mut I2C_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable dual addressing mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).OAR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x1 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable dual addressing mode */
        ::core::ptr::write_volatile(&mut (*I2Cx).OAR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).OAR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfffe as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C general call feature.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C General call.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GeneralCallCmd(mut I2Cx: *mut I2C_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable generall call */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x40 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable generall call */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xffbf as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C interrupts.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_IT: specifies the I2C interrupts sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg I2C_IT_BUF: Buffer interrupt mask
  *     @arg I2C_IT_EVT: Event interrupt mask
  *     @arg I2C_IT_ERR: Error interrupt mask
  * @param  NewState: new state of the specified I2C interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ITConfig(mut I2Cx: *mut I2C_TypeDef,
                                      mut I2C_IT: uint16_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C interrupts */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         I2C_IT as libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C interrupts */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(I2C_IT as libc::c_int) as uint16_t
                                             as libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
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
    ::core::ptr::write_volatile(&mut (*I2Cx).DR as *mut uint16_t,
                                Data as uint16_t);
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
    return (*I2Cx).DR as uint8_t;
}
/* *
  * @brief  Transmits the address byte to select the slave device.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  Address: specifies the slave address which will be transmitted
  * @param  I2C_Direction: specifies whether the I2C device will be a
  *   Transmitter or a Receiver. This parameter can be one of the following values
  *     @arg I2C_Direction_Transmitter: Transmitter mode
  *     @arg I2C_Direction_Receiver: Receiver mode
  * @retval None.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_Send7bitAddress(mut I2Cx: *mut I2C_TypeDef,
                                             mut Address: uint8_t,
                                             mut I2C_Direction: uint8_t) {
    /* Check the parameters */
    /* Test on the direction to set/reset the read/write bit */
    if I2C_Direction as libc::c_int !=
           0 as libc::c_int as uint8_t as libc::c_int {
        /* Set the address bit0 for read */
        Address =
            (Address as libc::c_int |
                 0x1 as libc::c_int as uint16_t as libc::c_int) as uint8_t
    } else {
        /* Reset the address bit0 for write */
        Address =
            (Address as libc::c_int &
                 0xfffe as libc::c_int as uint16_t as libc::c_int) as uint8_t
    }
    /* Send the address */
    ::core::ptr::write_volatile(&mut (*I2Cx).DR as *mut uint16_t,
                                Address as uint16_t);
}
/* *
  * @brief  Reads the specified I2C register and returns its value.
  * @param  I2C_Register: specifies the register to read.
  *   This parameter can be one of the following values:
  *     @arg I2C_Register_CR1:  CR1 register.
  *     @arg I2C_Register_CR2:   CR2 register.
  *     @arg I2C_Register_OAR1:  OAR1 register.
  *     @arg I2C_Register_OAR2:  OAR2 register.
  *     @arg I2C_Register_DR:    DR register.
  *     @arg I2C_Register_SR1:   SR1 register.
  *     @arg I2C_Register_SR2:   SR2 register.
  *     @arg I2C_Register_CCR:   CCR register.
  *     @arg I2C_Register_TRISE: TRISE register.
  * @retval The value of the read register.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ReadRegister(mut I2Cx: *mut I2C_TypeDef,
                                          mut I2C_Register: uint8_t)
 -> uint16_t {
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
    return *(tmp as *mut uint16_t);
}
/* *
  * @brief  Enables or disables the specified I2C software reset.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C software reset.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SoftwareResetCmd(mut I2Cx: *mut I2C_TypeDef,
                                              mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Peripheral under reset */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x8000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Peripheral not under reset */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0x7fff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Selects the specified I2C NACK position in master receiver mode.
  *         This function is useful in I2C Master Receiver mode when the number
  *         of data to be received is equal to 2. In this case, this function 
  *         should be called (with parameter I2C_NACKPosition_Next) before data 
  *         reception starts,as described in the 2-byte reception procedure 
  *         recommended in Reference Manual in Section: Master receiver.                
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_NACKPosition: specifies the NACK position. 
  *   This parameter can be one of the following values:
  *     @arg I2C_NACKPosition_Next: indicates that the next byte will be the last
  *          received byte.  
  *     @arg I2C_NACKPosition_Current: indicates that current byte is the last 
  *          received byte.
  *            
  * @note    This function configures the same bit (POS) as I2C_PECPositionConfig() 
  *          but is intended to be used in I2C mode while I2C_PECPositionConfig() 
  *          is intended to used in SMBUS mode. 
  *            
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_NACKPositionConfig(mut I2Cx: *mut I2C_TypeDef,
                                                mut I2C_NACKPosition:
                                                    uint16_t) {
    /* Check the parameters */
    /* Check the input parameter */
    if I2C_NACKPosition as libc::c_int ==
           0x800 as libc::c_int as uint16_t as libc::c_int {
        /* Next byte in shift register is the last received byte */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x800 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Current byte in shift register is the last received byte */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xf7ff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Drives the SMBusAlert pin high or low for the specified I2C.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_SMBusAlert: specifies SMBAlert pin level. 
  *   This parameter can be one of the following values:
  *     @arg I2C_SMBusAlert_Low: SMBAlert pin driven low
  *     @arg I2C_SMBusAlert_High: SMBAlert pin driven high
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_SMBusAlertConfig(mut I2Cx: *mut I2C_TypeDef,
                                              mut I2C_SMBusAlert: uint16_t) {
    /* Check the parameters */
    if I2C_SMBusAlert as libc::c_int ==
           0x2000 as libc::c_int as uint16_t as libc::c_int {
        /* Drive the SMBusAlert pin Low */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Drive the SMBusAlert pin High  */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xdfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C PEC transfer.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2C PEC transmission.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_TransmitPEC(mut I2Cx: *mut I2C_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C PEC transmission */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x1000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C PEC transmission */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xefff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Selects the specified I2C PEC position.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_PECPosition: specifies the PEC position. 
  *   This parameter can be one of the following values:
  *     @arg I2C_PECPosition_Next: indicates that the next byte is PEC
  *     @arg I2C_PECPosition_Current: indicates that current byte is PEC
  *       
  * @note    This function configures the same bit (POS) as I2C_NACKPositionConfig()
  *          but is intended to be used in SMBUS mode while I2C_NACKPositionConfig() 
  *          is intended to used in I2C mode.
  *               
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_PECPositionConfig(mut I2Cx: *mut I2C_TypeDef,
                                               mut I2C_PECPosition:
                                                   uint16_t) {
    /* Check the parameters */
    if I2C_PECPosition as libc::c_int ==
           0x800 as libc::c_int as uint16_t as libc::c_int {
        /* Next byte in shift register is PEC */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x800 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Current byte in shift register is PEC */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xf7ff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the PEC value calculation of the transferred bytes.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx PEC value calculation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_CalculatePEC(mut I2Cx: *mut I2C_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C PEC calculation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x20 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C PEC calculation */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xffdf as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Returns the PEC value for the specified I2C.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @retval The PEC value.
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetPEC(mut I2Cx: *mut I2C_TypeDef) -> uint8_t {
    /* Check the parameters */
    /* Return the selected I2C PEC value */
    return ((*I2Cx).SR2 as libc::c_int >> 8 as libc::c_int) as uint8_t;
}
/* *
  * @brief  Enables or disables the specified I2C ARP.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx ARP. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ARPCmd(mut I2Cx: *mut I2C_TypeDef,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C ARP */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x10 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C ARP */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xffef as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified I2C Clock stretching.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  NewState: new state of the I2Cx Clock stretching.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_StretchClockCmd(mut I2Cx: *mut I2C_TypeDef,
                                             mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint == DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected I2C Clock stretching */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x80 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected I2C Clock stretching */
        ::core::ptr::write_volatile(&mut (*I2Cx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xff7f as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Selects the specified I2C fast mode duty cycle.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_DutyCycle: specifies the fast mode duty cycle.
  *   This parameter can be one of the following values:
  *     @arg I2C_DutyCycle_2: I2C fast mode Tlow/Thigh = 2
  *     @arg I2C_DutyCycle_16_9: I2C fast mode Tlow/Thigh = 16/9
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_FastModeDutyCycleConfig(mut I2Cx:
                                                         *mut I2C_TypeDef,
                                                     mut I2C_DutyCycle:
                                                         uint16_t) {
    /* Check the parameters */
    if I2C_DutyCycle as libc::c_int !=
           0x4000 as libc::c_int as uint16_t as libc::c_int {
        /* I2C fast mode Tlow/Thigh=2 */
        ::core::ptr::write_volatile(&mut (*I2Cx).CCR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CCR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xbfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* I2C fast mode Tlow/Thigh=16/9 */
        ::core::ptr::write_volatile(&mut (*I2Cx).CCR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*I2Cx).CCR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
 * @brief
 ****************************************************************************************
 *
 *                         I2C State Monitoring Functions
 *                       
 ****************************************************************************************   
 * This I2C driver provides three different ways for I2C state monitoring
 *  depending on the application requirements and constraints:
 *        
 *  
 * 1) Basic state monitoring:
 *    Using I2C_CheckEvent() function:
 *    It compares the status registers (SR1 and SR2) content to a given event
 *    (can be the combination of one or more flags).
 *    It returns SUCCESS if the current status includes the given flags 
 *    and returns ERROR if one or more flags are missing in the current status.
 *    - When to use:
 *      - This function is suitable for most applications as well as for startup 
 *      activity since the events are fully described in the product reference manual 
 *      (RM0008).
 *      - It is also suitable for users who need to define their own events.
 *    - Limitations:
 *      - If an error occurs (ie. error flags are set besides to the monitored flags),
 *        the I2C_CheckEvent() function may return SUCCESS despite the communication
 *        hold or corrupted real state. 
 *        In this case, it is advised to use error interrupts to monitor the error
 *        events and handle them in the interrupt IRQ handler.
 *        
 *        @note 
 *        For error management, it is advised to use the following functions:
 *          - I2C_ITConfig() to configure and enable the error interrupts (I2C_IT_ERR).
 *          - I2Cx_ER_IRQHandler() which is called when the error interrupt occurs.
 *            Where x is the peripheral instance (I2C1, I2C2 ...)
 *          - I2C_GetFlagStatus() or I2C_GetITStatus() to be called into I2Cx_ER_IRQHandler() 
 *            in order to determine which error occured.
 *          - I2C_ClearFlag() or I2C_ClearITPendingBit() and/or I2C_SoftwareResetCmd()
 *            and/or I2C_GenerateStop() in order to clear the error flag and source,
 *            and return to correct communication status.
 *            
 *
 *  2) Advanced state monitoring:
 *     Using the function I2C_GetLastEvent() which returns the image of both status 
 *     registers in a single word (uint32_t) (Status Register 2 value is shifted left 
 *     by 16 bits and concatenated to Status Register 1).
 *     - When to use:
 *       - This function is suitable for the same applications above but it allows to
 *         overcome the mentioned limitation of I2C_GetFlagStatus() function.
 *         The returned value could be compared to events already defined in the 
 *         library (stm32f10x_i2c.h) or to custom values defined by user.
 *       - This function is suitable when multiple flags are monitored at the same time.
 *       - At the opposite of I2C_CheckEvent() function, this function allows user to
 *         choose when an event is accepted (when all events flags are set and no 
 *         other flags are set or just when the needed flags are set like 
 *         I2C_CheckEvent() function).
 *     - Limitations:
 *       - User may need to define his own events.
 *       - Same remark concerning the error management is applicable for this 
 *         function if user decides to check only regular communication flags (and 
 *         ignores error flags).
 *     
 *
 *  3) Flag-based state monitoring:
 *     Using the function I2C_GetFlagStatus() which simply returns the status of 
 *     one single flag (ie. I2C_FLAG_RXNE ...). 
 *     - When to use:
 *        - This function could be used for specific applications or in debug phase.
 *        - It is suitable when only one flag checking is needed (most I2C events 
 *          are monitored through multiple flags).
 *     - Limitations: 
 *        - When calling this function, the Status register is accessed. Some flags are
 *          cleared when the status register is accessed. So checking the status
 *          of one Flag, may clear other ones.
 *        - Function may need to be called twice or more in order to monitor one 
 *          single event.
 *
 *  For detailed description of Events, please refer to section I2C_Events in 
 *  stm32f10x_i2c.h file.
 *  
 */
/* *
 * 
 *  1) Basic state monitoring
 *******************************************************************************
 */
/* *
  * @brief  Checks whether the last I2Cx Event is equal to the one passed
  *   as parameter.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_EVENT: specifies the event to be checked. 
  *   This parameter can be one of the following values:
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED           : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED              : EV1
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED     : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED        : EV1
  *     @arg I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED            : EV1
  *     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED                         : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)      : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)    : EV2
  *     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED                      : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)   : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) : EV3
  *     @arg I2C_EVENT_SLAVE_ACK_FAILURE                           : EV3_2
  *     @arg I2C_EVENT_SLAVE_STOP_DETECTED                         : EV4
  *     @arg I2C_EVENT_MASTER_MODE_SELECT                          : EV5
  *     @arg I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6     
  *     @arg I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
  *     @arg I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
  *     @arg I2C_EVENT_MASTER_MODE_ADDRESS10                       : EV9
  *     
  * @note: For detailed description of Events, please refer to section 
  *    I2C_Events in stm32f10x_i2c.h file.
  *    
  * @retval An ErrorStatus enumeration value:
  * - SUCCESS: Last event is equal to the I2C_EVENT
  * - ERROR: Last event is different from the I2C_EVENT
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_CheckEvent(mut I2Cx: *mut I2C_TypeDef,
                                        mut I2C_EVENT: uint32_t)
 -> ErrorStatus {
    let mut lastevent: uint32_t = 0 as libc::c_int as uint32_t;
    let mut flag1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut flag2: uint32_t = 0 as libc::c_int as uint32_t;
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    /* Read the I2Cx status register */
    flag1 = (*I2Cx).SR1 as uint32_t;
    flag2 = (*I2Cx).SR2 as uint32_t;
    flag2 = flag2 << 16 as libc::c_int;
    /* Get the last event value from I2C status register */
    lastevent = (flag1 | flag2) & 0xffffff as libc::c_int as uint32_t;
    /* Check whether the last event contains the I2C_EVENT */
    if lastevent & I2C_EVENT == I2C_EVENT {
        /* SUCCESS: last event is equal to I2C_EVENT */
        status = SUCCESS
    } else {
        /* ERROR: last event is different from I2C_EVENT */
        status = ERROR
    }
    /* Return status */
    return status;
}
/* *
 * 
 *  2) Advanced state monitoring
 *******************************************************************************
 */
/* *
  * @brief  Returns the last I2Cx Event.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  *     
  * @note: For detailed description of Events, please refer to section 
  *    I2C_Events in stm32f10x_i2c.h file.
  *    
  * @retval The last event
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetLastEvent(mut I2Cx: *mut I2C_TypeDef)
 -> uint32_t {
    let mut lastevent: uint32_t = 0 as libc::c_int as uint32_t;
    let mut flag1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut flag2: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Read the I2Cx status register */
    flag1 = (*I2Cx).SR1 as uint32_t;
    flag2 = (*I2Cx).SR2 as uint32_t;
    flag2 = flag2 << 16 as libc::c_int;
    /* Get the last event value from I2C status register */
    lastevent = (flag1 | flag2) & 0xffffff as libc::c_int as uint32_t;
    /* Return status */
    return lastevent;
}
/* *
 * 
 *  3) Flag-based state monitoring
 *******************************************************************************
 */
/* *
  * @brief  Checks whether the specified I2C flag is set or not.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *     @arg I2C_FLAG_DUALF: Dual flag (Slave mode)
  *     @arg I2C_FLAG_SMBHOST: SMBus host header (Slave mode)
  *     @arg I2C_FLAG_SMBDEFAULT: SMBus default header (Slave mode)
  *     @arg I2C_FLAG_GENCALL: General call header flag (Slave mode)
  *     @arg I2C_FLAG_TRA: Transmitter/Receiver flag
  *     @arg I2C_FLAG_BUSY: Bus busy flag
  *     @arg I2C_FLAG_MSL: Master/Slave flag
  *     @arg I2C_FLAG_SMBALERT: SMBus Alert flag
  *     @arg I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
  *     @arg I2C_FLAG_PECERR: PEC error in reception flag
  *     @arg I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
  *     @arg I2C_FLAG_AF: Acknowledge failure flag
  *     @arg I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
  *     @arg I2C_FLAG_BERR: Bus error flag
  *     @arg I2C_FLAG_TXE: Data register empty flag (Transmitter)
  *     @arg I2C_FLAG_RXNE: Data register not empty (Receiver) flag
  *     @arg I2C_FLAG_STOPF: Stop detection flag (Slave mode)
  *     @arg I2C_FLAG_ADD10: 10-bit header sent flag (Master mode)
  *     @arg I2C_FLAG_BTF: Byte transfer finished flag
  *     @arg I2C_FLAG_ADDR: Address sent flag (Master mode) "ADSL"
  *   Address matched flag (Slave mode)"ENDA"
  *     @arg I2C_FLAG_SB: Start bit flag (Master mode)
  * @retval The new state of I2C_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetFlagStatus(mut I2Cx: *mut I2C_TypeDef,
                                           mut I2C_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    let mut i2creg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut i2cxbase: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the I2Cx peripheral base address */
    ::core::ptr::write_volatile(&mut i2cxbase as *mut uint32_t,
                                I2Cx as uint32_t);
    /* Read flag register index */
    ::core::ptr::write_volatile(&mut i2creg as *mut uint32_t,
                                I2C_FLAG >> 28 as libc::c_int);
    /* Get bit[23:0] of the flag */
    I2C_FLAG &= 0xffffff as libc::c_int as uint32_t;
    if i2creg != 0 as libc::c_int as libc::c_uint {
        /* Get the I2Cx SR1 register address */
        ::core::ptr::write_volatile(&mut i2cxbase as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&i2cxbase
                                                                                as
                                                                                *const uint32_t)
                                         as
                                         libc::c_uint).wrapping_add(0x14 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                        as uint32_t as uint32_t)
    } else {
        /* Flag in I2Cx SR2 Register */
        I2C_FLAG = I2C_FLAG >> 16 as libc::c_int;
        /* Get the I2Cx SR2 register address */
        ::core::ptr::write_volatile(&mut i2cxbase as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&i2cxbase
                                                                                as
                                                                                *const uint32_t)
                                         as
                                         libc::c_uint).wrapping_add(0x18 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                        as uint32_t as uint32_t)
    }
    if *(i2cxbase as *mut uint32_t) & I2C_FLAG !=
           RESET as libc::c_int as uint32_t {
        /* I2C_FLAG is set */
        bitstatus = SET
    } else {
        /* I2C_FLAG is reset */
        bitstatus = RESET
    }
    /* Return the I2C_FLAG status */
    return bitstatus;
}
/* *
  * @brief  Clears the I2Cx's pending flags.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_FLAG: specifies the flag to clear. 
  *   This parameter can be any combination of the following values:
  *     @arg I2C_FLAG_SMBALERT: SMBus Alert flag
  *     @arg I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
  *     @arg I2C_FLAG_PECERR: PEC error in reception flag
  *     @arg I2C_FLAG_OVR: Overrun/Underrun flag (Slave mode)
  *     @arg I2C_FLAG_AF: Acknowledge failure flag
  *     @arg I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
  *     @arg I2C_FLAG_BERR: Bus error flag
  *   
  * @note
  *   - STOPF (STOP detection) is cleared by software sequence: a read operation 
  *     to I2C_SR1 register (I2C_GetFlagStatus()) followed by a write operation 
  *     to I2C_CR1 register (I2C_Cmd() to re-enable the I2C peripheral).
  *   - ADD10 (10-bit header sent) is cleared by software sequence: a read 
  *     operation to I2C_SR1 (I2C_GetFlagStatus()) followed by writing the 
  *     second byte of the address in DR register.
  *   - BTF (Byte Transfer Finished) is cleared by software sequence: a read 
  *     operation to I2C_SR1 register (I2C_GetFlagStatus()) followed by a 
  *     read/write to I2C_DR register (I2C_SendData()).
  *   - ADDR (Address sent) is cleared by software sequence: a read operation to 
  *     I2C_SR1 register (I2C_GetFlagStatus()) followed by a read operation to 
  *     I2C_SR2 register ((void)(I2Cx->SR2)).
  *   - SB (Start Bit) is cleared software sequence: a read operation to I2C_SR1
  *     register (I2C_GetFlagStatus()) followed by a write operation to I2C_DR
  *     register  (I2C_SendData()).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ClearFlag(mut I2Cx: *mut I2C_TypeDef,
                                       mut I2C_FLAG: uint32_t) {
    let mut flagpos: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the I2C flag position */
    flagpos = I2C_FLAG & 0xffffff as libc::c_int as uint32_t;
    /* Clear the selected I2C flag */
    ::core::ptr::write_volatile(&mut (*I2Cx).SR1 as *mut uint16_t,
                                !flagpos as uint16_t);
}
/* *
  * @brief  Checks whether the specified I2C interrupt has occurred or not.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg I2C_IT_SMBALERT: SMBus Alert flag
  *     @arg I2C_IT_TIMEOUT: Timeout or Tlow error flag
  *     @arg I2C_IT_PECERR: PEC error in reception flag
  *     @arg I2C_IT_OVR: Overrun/Underrun flag (Slave mode)
  *     @arg I2C_IT_AF: Acknowledge failure flag
  *     @arg I2C_IT_ARLO: Arbitration lost flag (Master mode)
  *     @arg I2C_IT_BERR: Bus error flag
  *     @arg I2C_IT_TXE: Data register empty flag (Transmitter)
  *     @arg I2C_IT_RXNE: Data register not empty (Receiver) flag
  *     @arg I2C_IT_STOPF: Stop detection flag (Slave mode)
  *     @arg I2C_IT_ADD10: 10-bit header sent flag (Master mode)
  *     @arg I2C_IT_BTF: Byte transfer finished flag
  *     @arg I2C_IT_ADDR: Address sent flag (Master mode) "ADSL"
  *                       Address matched flag (Slave mode)"ENDAD"
  *     @arg I2C_IT_SB: Start bit flag (Master mode)
  * @retval The new state of I2C_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_GetITStatus(mut I2Cx: *mut I2C_TypeDef,
                                         mut I2C_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut enablestatus: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Check if the interrupt source is enabled or not */
    enablestatus =
        (I2C_IT & 0x7000000 as libc::c_int as uint32_t) >> 16 as libc::c_int &
            (*I2Cx).CR2 as libc::c_uint;
    /* Get bit[23:0] of the flag */
    I2C_IT &= 0xffffff as libc::c_int as uint32_t;
    /* Check the status of the specified I2C flag */
    if (*I2Cx).SR1 as libc::c_uint & I2C_IT !=
           RESET as libc::c_int as uint32_t && enablestatus != 0 {
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
  * @file    stm32f10x_i2c.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the I2C firmware 
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
/* * @addtogroup I2C
  * @{
  */
/* * @defgroup I2C_Exported_Types
  * @{
  */
/* * 
  * @brief  I2C Init structure definition  
  */
/* !< Specifies the clock frequency.
                                         This parameter must be set to a value lower than 400kHz */
/* !< Specifies the I2C mode.
                                         This parameter can be a value of @ref I2C_mode */
/* !< Specifies the I2C fast mode duty cycle.
                                         This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */
/* !< Specifies the first device own address.
                                         This parameter can be a 7-bit or 10-bit address. */
/* !< Enables or disables the acknowledgement.
                                         This parameter can be a value of @ref I2C_acknowledgement */
/* !< Specifies if 7-bit or 10-bit address is acknowledged.
                                         This parameter can be a value of @ref I2C_acknowledged_address */
/* *
  * @}
  */
/* * @defgroup I2C_Exported_Constants
  * @{
  */
/* * @defgroup I2C_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_duty_cycle_in_fast_mode 
  * @{
  */
/* !< I2C fast mode Tlow/Thigh = 16/9 */
/* !< I2C fast mode Tlow/Thigh = 2 */
/* *
  * @}
  */
/* * @defgroup I2C_acknowledgement
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
/* * @defgroup I2C_acknowledged_address 
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
/* * @defgroup I2C_SMBus_alert_pin_level 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_PEC_position 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_NCAK_position 
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
  * @brief  SR2 register flags  
  */
/* * 
  * @brief  SR1 register flags  
  */
/* *
  * @}
  */
/* * @defgroup I2C_Events 
  * @{
  */
/*========================================
     
                     I2C Master Events (Events grouped in order of communication)
                                                        ==========================================*/
/* * 
  * @brief  Communication start
  * 
  * After sending the START condition (I2C_GenerateSTART() function) the master 
  * has to wait for this event. It means that the Start condition has been correctly 
  * released on the I2C bus (the bus is free, no other devices is communicating).
  * 
  */
/* --EV5 */
/* BUSY, MSL and SB flag */
/* * 
  * @brief  Address Acknowledge
  * 
  * After checking on EV5 (start condition correctly released on the bus), the 
  * master sends the address of the slave(s) with which it will communicate 
  * (I2C_Send7bitAddress() function, it also determines the direction of the communication: 
  * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges 
  * his address. If an acknowledge is sent on the bus, one of the following events will 
  * be set:
  * 
  *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED 
  *     event is set.
  *  
  *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 
  *     is set
  *  
  *  3) In case of 10-Bit addressing mode, the master (just after generating the START 
  *  and checking on EV5) has to send the header of 10-bit addressing mode (I2C_SendData() 
  *  function). Then master should wait on EV9. It means that the 10-bit addressing 
  *  header has been correctly sent on the bus. Then master should send the second part of 
  *  the 10-bit address (LSB) using the function I2C_Send7bitAddress(). Then master 
  *  should wait for event EV6. 
  *     
  */
/* --EV6 */
/* BUSY, MSL, ADDR, TXE and TRA flags */
/* BUSY, MSL and ADDR flags */
/* --EV9 */
/* BUSY, MSL and ADD10 flags */
/* * 
  * @brief Communication events
  * 
  * If a communication is established (START condition generated and slave address 
  * acknowledged) then the master has to check on one of the following events for 
  * communication procedures:
  *  
  * 1) Master Receiver mode: The master has to wait on the event EV7 then to read 
  *    the data received from the slave (I2C_ReceiveData() function).
  * 
  * 2) Master Transmitter mode: The master has to send data (I2C_SendData() 
  *    function) then to wait on event EV8 or EV8_2.
  *    These two events are similar: 
  *     - EV8 means that the data has been written in the data register and is 
  *       being shifted out.
  *     - EV8_2 means that the data has been physically shifted out and output 
  *       on the bus.
  *     In most cases, using EV8 is sufficient for the application.
  *     Using EV8_2 leads to a slower communication but ensure more reliable test.
  *     EV8_2 is also more suitable than EV8 for testing on the last data transmission 
  *     (before Stop condition generation).
  *     
  *  @note In case the  user software does not guarantee that this event EV7 is 
  *  managed before the current byte end of transfer, then user may check on EV7 
  *  and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
  *  In this case the communication may be slower.
  * 
  */
/* Master RECEIVER mode -----------------------------*/ 
/* --EV7 */
/* BUSY, MSL and RXNE flags */
/* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
/* TRA, BUSY, MSL, TXE flags */
/* --EV8_2 */
/* TRA, BUSY, MSL, TXE and BTF flags */
/*========================================
     
                     I2C Slave Events (Events grouped in order of communication)
                                                        ==========================================*/
/* * 
  * @brief  Communication start events
  * 
  * Wait on one of these events at the start of the communication. It means that 
  * the I2C peripheral detected a Start condition on the bus (generated by master 
  * device) followed by the peripheral address. The peripheral generates an ACK 
  * condition on the bus (if the acknowledge feature is enabled through function 
  * I2C_AcknowledgeConfig()) and the events listed above are set :
  *  
  * 1) In normal case (only one address managed by the slave), when the address 
  *   sent by the master matches the own address of the peripheral (configured by 
  *   I2C_OwnAddress1 field) the I2C_EVENT_SLAVE_XXX_ADDRESS_MATCHED event is set 
  *   (where XXX could be TRANSMITTER or RECEIVER).
  *    
  * 2) In case the address sent by the master matches the second address of the 
  *   peripheral (configured by the function I2C_OwnAddress2Config() and enabled 
  *   by the function I2C_DualAddressCmd()) the events I2C_EVENT_SLAVE_XXX_SECONDADDRESS_MATCHED 
  *   (where XXX could be TRANSMITTER or RECEIVER) are set.
  *   
  * 3) In case the address sent by the master is General Call (address 0x00) and 
  *   if the General Call is enabled for the peripheral (using function I2C_GeneralCallCmd()) 
  *   the following event is set I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED.   
  * 
  */
/* --EV1  (all the events below are variants of EV1) */   
/* 1) Case of One Single Address managed by the slave */
/* BUSY and ADDR flags */
/* TRA, BUSY, TXE and ADDR flags */
/* 2) Case of Dual address managed by the slave */
/* DUALF and BUSY flags */
/* DUALF, TRA, BUSY and TXE flags */
/* 3) Case of General Call enabled for the slave */
/* GENCALL and BUSY flags */
/* * 
  * @brief  Communication events
  * 
  * Wait on one of these events when EV1 has already been checked and: 
  * 
  * - Slave RECEIVER mode:
  *     - EV2: When the application is expecting a data byte to be received. 
  *     - EV4: When the application is expecting the end of the communication: master 
  *       sends a stop condition and data transmission is stopped.
  *    
  * - Slave Transmitter mode:
  *    - EV3: When a byte has been transmitted by the slave and the application is expecting 
  *      the end of the byte transmission. The two events I2C_EVENT_SLAVE_BYTE_TRANSMITTED and
  *      I2C_EVENT_SLAVE_BYTE_TRANSMITTING are similar. The second one can optionally be 
  *      used when the user software doesn't guarantee the EV3 is managed before the
  *      current byte end of transfer.
  *    - EV3_2: When the master sends a NACK in order to tell slave that data transmission 
  *      shall end (before sending the STOP condition). In this case slave has to stop sending 
  *      data bytes and expect a Stop condition on the bus.
  *      
  *  @note In case the  user software does not guarantee that the event EV2 is 
  *  managed before the current byte end of transfer, then user may check on EV2 
  *  and BTF flag at the same time (ie. (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_BTF)).
  * In this case the communication may be slower.
  *
  */
/* Slave RECEIVER mode --------------------------*/ 
/* --EV2 */
/* BUSY and RXNE flags */
/* --EV4  */
/* STOPF flag */
/* Slave TRANSMITTER mode -----------------------*/
/* --EV3 */
/* TRA, BUSY, TXE and BTF flags */
/* TRA, BUSY and TXE flags */
/* --EV3_2 */
/* AF flag */
/*===========================      End of Events Description           ==========================================*/
/* *
  * @}
  */
/* * @defgroup I2C_own_address1 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_clock_speed 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup I2C_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2C_Exported_Functions
  * @{
  */
/* *
 * @brief
 ****************************************************************************************
 *
 *                         I2C State Monitoring Functions
 *                       
 ****************************************************************************************   
 * This I2C driver provides three different ways for I2C state monitoring
 *  depending on the application requirements and constraints:
 *        
 *  
 * 1) Basic state monitoring:
 *    Using I2C_CheckEvent() function:
 *    It compares the status registers (SR1 and SR2) content to a given event
 *    (can be the combination of one or more flags).
 *    It returns SUCCESS if the current status includes the given flags 
 *    and returns ERROR if one or more flags are missing in the current status.
 *    - When to use:
 *      - This function is suitable for most applications as well as for startup 
 *      activity since the events are fully described in the product reference manual 
 *      (RM0008).
 *      - It is also suitable for users who need to define their own events.
 *    - Limitations:
 *      - If an error occurs (ie. error flags are set besides to the monitored flags),
 *        the I2C_CheckEvent() function may return SUCCESS despite the communication
 *        hold or corrupted real state. 
 *        In this case, it is advised to use error interrupts to monitor the error
 *        events and handle them in the interrupt IRQ handler.
 *        
 *        @note 
 *        For error management, it is advised to use the following functions:
 *          - I2C_ITConfig() to configure and enable the error interrupts (I2C_IT_ERR).
 *          - I2Cx_ER_IRQHandler() which is called when the error interrupt occurs.
 *            Where x is the peripheral instance (I2C1, I2C2 ...)
 *          - I2C_GetFlagStatus() or I2C_GetITStatus() to be called into I2Cx_ER_IRQHandler()
 *            in order to determine which error occurred.
 *          - I2C_ClearFlag() or I2C_ClearITPendingBit() and/or I2C_SoftwareResetCmd()
 *            and/or I2C_GenerateStop() in order to clear the error flag and source,
 *            and return to correct communication status.
 *            
 *
 *  2) Advanced state monitoring:
 *     Using the function I2C_GetLastEvent() which returns the image of both status 
 *     registers in a single word (uint32_t) (Status Register 2 value is shifted left 
 *     by 16 bits and concatenated to Status Register 1).
 *     - When to use:
 *       - This function is suitable for the same applications above but it allows to
 *         overcome the limitations of I2C_GetFlagStatus() function (see below).
 *         The returned value could be compared to events already defined in the 
 *         library (stm32f10x_i2c.h) or to custom values defined by user.
 *       - This function is suitable when multiple flags are monitored at the same time.
 *       - At the opposite of I2C_CheckEvent() function, this function allows user to
 *         choose when an event is accepted (when all events flags are set and no 
 *         other flags are set or just when the needed flags are set like 
 *         I2C_CheckEvent() function).
 *     - Limitations:
 *       - User may need to define his own events.
 *       - Same remark concerning the error management is applicable for this 
 *         function if user decides to check only regular communication flags (and 
 *         ignores error flags).
 *     
 *
 *  3) Flag-based state monitoring:
 *     Using the function I2C_GetFlagStatus() which simply returns the status of 
 *     one single flag (ie. I2C_FLAG_RXNE ...). 
 *     - When to use:
 *        - This function could be used for specific applications or in debug phase.
 *        - It is suitable when only one flag checking is needed (most I2C events 
 *          are monitored through multiple flags).
 *     - Limitations: 
 *        - When calling this function, the Status register is accessed. Some flags are
 *          cleared when the status register is accessed. So checking the status
 *          of one Flag, may clear other ones.
 *        - Function may need to be called twice or more in order to monitor one 
 *          single event.
 *            
 */
/* *
 * 
 *  1) Basic state monitoring
 *******************************************************************************
 */
/* *
 * 
 *  2) Advanced state monitoring
 *******************************************************************************
 */
/* *
 * 
 *  3) Flag-based state monitoring
 *******************************************************************************
 */
/* *
 *
 *******************************************************************************
 */
/* *
  * @brief  Clears the I2Cx�s interrupt pending bits.
  * @param  I2Cx: where x can be 1 or 2 to select the I2C peripheral.
  * @param  I2C_IT: specifies the interrupt pending bit to clear. 
  *   This parameter can be any combination of the following values:
  *     @arg I2C_IT_SMBALERT: SMBus Alert interrupt
  *     @arg I2C_IT_TIMEOUT: Timeout or Tlow error interrupt
  *     @arg I2C_IT_PECERR: PEC error in reception  interrupt
  *     @arg I2C_IT_OVR: Overrun/Underrun interrupt (Slave mode)
  *     @arg I2C_IT_AF: Acknowledge failure interrupt
  *     @arg I2C_IT_ARLO: Arbitration lost interrupt (Master mode)
  *     @arg I2C_IT_BERR: Bus error interrupt
  *   
  * @note
  *   - STOPF (STOP detection) is cleared by software sequence: a read operation 
  *     to I2C_SR1 register (I2C_GetITStatus()) followed by a write operation to 
  *     I2C_CR1 register (I2C_Cmd() to re-enable the I2C peripheral).
  *   - ADD10 (10-bit header sent) is cleared by software sequence: a read 
  *     operation to I2C_SR1 (I2C_GetITStatus()) followed by writing the second 
  *     byte of the address in I2C_DR register.
  *   - BTF (Byte Transfer Finished) is cleared by software sequence: a read 
  *     operation to I2C_SR1 register (I2C_GetITStatus()) followed by a 
  *     read/write to I2C_DR register (I2C_SendData()).
  *   - ADDR (Address sent) is cleared by software sequence: a read operation to 
  *     I2C_SR1 register (I2C_GetITStatus()) followed by a read operation to 
  *     I2C_SR2 register ((void)(I2Cx->SR2)).
  *   - SB (Start Bit) is cleared by software sequence: a read operation to 
  *     I2C_SR1 register (I2C_GetITStatus()) followed by a write operation to 
  *     I2C_DR register (I2C_SendData()).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2C_ClearITPendingBit(mut I2Cx: *mut I2C_TypeDef,
                                               mut I2C_IT: uint32_t) {
    let mut flagpos: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Get the I2C flag position */
    flagpos = I2C_IT & 0xffffff as libc::c_int as uint32_t;
    /* Clear the selected I2C flag */
    ::core::ptr::write_volatile(&mut (*I2Cx).SR1 as *mut uint16_t,
                                !flagpos as uint16_t);
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
