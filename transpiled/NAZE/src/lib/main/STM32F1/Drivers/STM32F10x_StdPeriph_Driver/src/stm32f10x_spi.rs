use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_GetClocksFreq(RCC_Clocks: *mut RCC_ClocksTypeDef);
    /* STM32F10X_CL */
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
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
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
pub struct SPI_InitTypeDef {
    pub SPI_Direction: uint16_t,
    pub SPI_Mode: uint16_t,
    pub SPI_DataSize: uint16_t,
    pub SPI_CPOL: uint16_t,
    pub SPI_CPHA: uint16_t,
    pub SPI_NSS: uint16_t,
    pub SPI_BaudRatePrescaler: uint16_t,
    pub SPI_FirstBit: uint16_t,
    pub SPI_CRCPolynomial: uint16_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2S_InitTypeDef {
    pub I2S_Mode: uint16_t,
    pub I2S_Standard: uint16_t,
    pub I2S_DataFormat: uint16_t,
    pub I2S_MCLKOutput: uint16_t,
    pub I2S_AudioFreq: uint32_t,
    pub I2S_CPOL: uint16_t,
}
/* *
  * @}
  */
/* * @defgroup SPI_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the SPIx peripheral registers to their default
  *         reset values (Affects also the I2Ss).
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_DeInit(mut SPIx: *mut SPI_TypeDef) {
    /* Check the parameters */
    if SPIx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x3000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut SPI_TypeDef {
        /* Enable SPI1 reset state */
        RCC_APB2PeriphResetCmd(0x1000 as libc::c_int as uint32_t, ENABLE);
        /* Release SPI1 from reset state */
        RCC_APB2PeriphResetCmd(0x1000 as libc::c_int as uint32_t, DISABLE);
    } else if SPIx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x3800 as libc::c_int as
                                                  libc::c_uint) as
                      *mut SPI_TypeDef {
        /* Enable SPI2 reset state */
        RCC_APB1PeriphResetCmd(0x4000 as libc::c_int as uint32_t, ENABLE);
        /* Release SPI2 from reset state */
        RCC_APB1PeriphResetCmd(0x4000 as libc::c_int as uint32_t, DISABLE);
    } else if SPIx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x3c00 as libc::c_int as
                                                  libc::c_uint) as
                      *mut SPI_TypeDef {
        /* Enable SPI3 reset state */
        RCC_APB1PeriphResetCmd(0x8000 as libc::c_int as uint32_t, ENABLE);
        /* Release SPI3 from reset state */
        RCC_APB1PeriphResetCmd(0x8000 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the SPI_InitStruct.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_InitStruct: pointer to a SPI_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_Init(mut SPIx: *mut SPI_TypeDef,
                                  mut SPI_InitStruct: *mut SPI_InitTypeDef) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* check the parameters */
    /*---------------------------- SPIx CR1 Configuration ------------------------*/
  /* Get the SPIx CR1 value */
    tmpreg = (*SPIx).CR1;
    /* Clear BIDIMode, BIDIOE, RxONLY, SSM, SSI, LSBFirst, BR, MSTR, CPOL and CPHA bits */
    tmpreg =
        (tmpreg as libc::c_int &
             0x3040 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler
     master/salve mode, CPOL and CPHA */
  /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
  /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
  /* Set LSBFirst bit according to SPI_FirstBit value */
  /* Set BR bits according to SPI_BaudRatePrescaler value */
  /* Set CPOL bit according to SPI_CPOL value */
  /* Set CPHA bit according to SPI_CPHA value */
    tmpreg =
        (tmpreg as libc::c_int |
             ((*SPI_InitStruct).SPI_Direction as uint32_t |
                  (*SPI_InitStruct).SPI_Mode as libc::c_uint |
                  (*SPI_InitStruct).SPI_DataSize as libc::c_uint |
                  (*SPI_InitStruct).SPI_CPOL as libc::c_uint |
                  (*SPI_InitStruct).SPI_CPHA as libc::c_uint |
                  (*SPI_InitStruct).SPI_NSS as libc::c_uint |
                  (*SPI_InitStruct).SPI_BaudRatePrescaler as libc::c_uint |
                  (*SPI_InitStruct).SPI_FirstBit as libc::c_uint) as uint16_t
                 as libc::c_int) as uint16_t;
    /* Write to SPIx CR1 */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t, tmpreg);
    /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xf7ff as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /*---------------------------- SPIx CRCPOLY Configuration --------------------*/
  /* Write to SPIx CRCPOLY */
    ::core::ptr::write_volatile(&mut (*SPIx).CRCPR as *mut uint16_t,
                                (*SPI_InitStruct).SPI_CRCPolynomial);
}
/* *
  * @brief  Initializes the SPIx peripheral according to the specified 
  *         parameters in the I2S_InitStruct.
  * @param  SPIx: where x can be  2 or 3 to select the SPI peripheral
  *         (configured in I2S mode).
  * @param  I2S_InitStruct: pointer to an I2S_InitTypeDef structure that
  *         contains the configuration information for the specified SPI peripheral
  *         configured in I2S mode.
  * @note
  *  The function calculates the optimal prescaler needed to obtain the most 
  *  accurate audio frequency (depending on the I2S clock source, the PLL values 
  *  and the product configuration). But in case the prescaler value is greater 
  *  than 511, the default value (0x02) will be configured instead.  *   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_Init(mut SPIx: *mut SPI_TypeDef,
                                  mut I2S_InitStruct: *mut I2S_InitTypeDef) {
    let mut tmpreg: uint16_t = 0 as libc::c_int as uint16_t;
    let mut i2sdiv: uint16_t = 2 as libc::c_int as uint16_t;
    let mut i2sodd: uint16_t = 0 as libc::c_int as uint16_t;
    let mut packetlength: uint16_t = 1 as libc::c_int as uint16_t;
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut RCC_Clocks: RCC_ClocksTypeDef =
        RCC_ClocksTypeDef{SYSCLK_Frequency: 0,
                          HCLK_Frequency: 0,
                          PCLK1_Frequency: 0,
                          PCLK2_Frequency: 0,
                          ADCCLK_Frequency: 0,};
    let mut sourceclock: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the I2S parameters */
    /*----------------------- SPIx I2SCFGR & I2SPR Configuration -----------------*/
  /* Clear I2SMOD, I2SE, I2SCFG, PCMSYNC, I2SSTD, CKPOL, DATLEN and CHLEN bits */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     0xf040 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    ::core::ptr::write_volatile(&mut (*SPIx).I2SPR as *mut uint16_t,
                                0x2 as libc::c_int as uint16_t);
    /* Get the I2SCFGR register value */
    tmpreg = (*SPIx).I2SCFGR;
    /* If the default value has to be written, reinitialize i2sdiv and i2sodd*/
    if (*I2S_InitStruct).I2S_AudioFreq == 2 as libc::c_int as uint32_t {
        i2sodd = 0 as libc::c_int as uint16_t;
        i2sdiv = 2 as libc::c_int as uint16_t
    } else {
        /* If the requested audio frequency is not the default, compute the prescaler */
        /* Check the frame length (For the Prescaler computing) */
        if (*I2S_InitStruct).I2S_DataFormat as libc::c_int ==
               0 as libc::c_int as uint16_t as libc::c_int {
            /* Packet length is 16 bits */
            packetlength = 1 as libc::c_int as uint16_t
        } else {
            /* Packet length is 32 bits */
            packetlength = 2 as libc::c_int as uint16_t
        }
        if SPIx as uint32_t ==
               (0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x3800 as libc::c_int as
                                               libc::c_uint) {
            /* The mask is relative to I2S2 */
            tmp = 0x20000 as libc::c_int as uint32_t
        } else {
            /* The mask is relative to I2S3 */
            tmp = 0x40000 as libc::c_int as uint32_t
        }
        RCC_GetClocksFreq(&mut RCC_Clocks);
        sourceclock = RCC_Clocks.SYSCLK_Frequency;
        if (*I2S_InitStruct).I2S_MCLKOutput as libc::c_int ==
               0x200 as libc::c_int as uint16_t as libc::c_int {
            /* Get the I2S clock source mask depending on the peripheral number */
            /* Check the I2S clock source configuration depending on the Device:
       Only Connectivity line devices have the PLL3 VCO clock */
            /* STM32F10X_HD */
            /* I2S Clock source is System clock: Get System Clock frequency */
            /* Get the source clock value: based on System Clock value */
            /* MCLK output is enabled */
            tmp =
                sourceclock.wrapping_div(256 as libc::c_int as
                                             libc::c_uint).wrapping_mul(10 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_div((*I2S_InitStruct).I2S_AudioFreq).wrapping_add(5
                                                                                                                                                         as
                                                                                                                                                         libc::c_int
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                    as uint16_t as uint32_t
        } else {
            /* MCLK output is disabled */
            tmp =
                sourceclock.wrapping_div((32 as libc::c_int *
                                              packetlength as libc::c_int) as
                                             libc::c_uint).wrapping_mul(10 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_div((*I2S_InitStruct).I2S_AudioFreq).wrapping_add(5
                                                                                                                                                         as
                                                                                                                                                         libc::c_int
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                    as uint16_t as uint32_t
        }
        tmp = tmp.wrapping_div(10 as libc::c_int as libc::c_uint);
        i2sodd =
            (tmp & 0x1 as libc::c_int as uint16_t as libc::c_uint) as
                uint16_t;
        i2sdiv =
            tmp.wrapping_sub(i2sodd as
                                 libc::c_uint).wrapping_div(2 as libc::c_int
                                                                as
                                                                libc::c_uint)
                as uint16_t;
        i2sodd = ((i2sodd as libc::c_int) << 8 as libc::c_int) as uint16_t
    }
    /* STM32F10X_CL */
    /* Compute the Real divider depending on the MCLK output state with a floating point */
    /* Remove the floating point */
    /* Check the parity of the divider */
    /* Compute the i2sdiv prescaler */
    /* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
    /* Test if the divider is 1 or 0 or greater than 0xFF */
    if (i2sdiv as libc::c_int) < 2 as libc::c_int ||
           i2sdiv as libc::c_int > 0xff as libc::c_int {
        /* Set the default values */
        i2sdiv = 2 as libc::c_int as uint16_t;
        i2sodd = 0 as libc::c_int as uint16_t
    }
    /* Write to SPIx I2SPR register the computed value */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SPR as *mut uint16_t,
                                (i2sdiv as libc::c_int |
                                     (i2sodd as libc::c_int |
                                          (*I2S_InitStruct).I2S_MCLKOutput as
                                              libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t);
    /* Configure the I2S with the SPI_InitStruct values */
    tmpreg =
        (tmpreg as libc::c_int |
             (0x800 as libc::c_int as uint16_t as libc::c_int |
                  ((*I2S_InitStruct).I2S_Mode as libc::c_int |
                       ((*I2S_InitStruct).I2S_Standard as libc::c_int |
                            ((*I2S_InitStruct).I2S_DataFormat as libc::c_int |
                                 (*I2S_InitStruct).I2S_CPOL as libc::c_int) as
                                uint16_t as libc::c_int) as uint16_t as
                           libc::c_int) as uint16_t as libc::c_int) as
                 uint16_t as libc::c_int) as uint16_t;
    /* Write to SPIx I2SCFGR */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                tmpreg);
}
/* *
  * @brief  Fills each SPI_InitStruct member with its default value.
  * @param  SPI_InitStruct : pointer to a SPI_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_StructInit(mut SPI_InitStruct:
                                            *mut SPI_InitTypeDef) {
    /*--------------- Reset SPI init structure parameters values -----------------*/
  /* Initialize the SPI_Direction member */
    (*SPI_InitStruct).SPI_Direction = 0 as libc::c_int as uint16_t;
    /* initialize the SPI_Mode member */
    (*SPI_InitStruct).SPI_Mode = 0 as libc::c_int as uint16_t;
    /* initialize the SPI_DataSize member */
    (*SPI_InitStruct).SPI_DataSize = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_CPOL member */
    (*SPI_InitStruct).SPI_CPOL = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_CPHA member */
    (*SPI_InitStruct).SPI_CPHA = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_NSS member */
    (*SPI_InitStruct).SPI_NSS = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_BaudRatePrescaler member */
    (*SPI_InitStruct).SPI_BaudRatePrescaler = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_FirstBit member */
    (*SPI_InitStruct).SPI_FirstBit = 0 as libc::c_int as uint16_t;
    /* Initialize the SPI_CRCPolynomial member */
    (*SPI_InitStruct).SPI_CRCPolynomial = 7 as libc::c_int as uint16_t;
}
/* *
  * @brief  Fills each I2S_InitStruct member with its default value.
  * @param  I2S_InitStruct : pointer to a I2S_InitTypeDef structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_StructInit(mut I2S_InitStruct:
                                            *mut I2S_InitTypeDef) {
    /*--------------- Reset I2S init structure parameters values -----------------*/
  /* Initialize the I2S_Mode member */
    (*I2S_InitStruct).I2S_Mode = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_Standard member */
    (*I2S_InitStruct).I2S_Standard = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_DataFormat member */
    (*I2S_InitStruct).I2S_DataFormat = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_MCLKOutput member */
    (*I2S_InitStruct).I2S_MCLKOutput = 0 as libc::c_int as uint16_t;
    /* Initialize the I2S_AudioFreq member */
    (*I2S_InitStruct).I2S_AudioFreq = 2 as libc::c_int as uint32_t;
    /* Initialize the I2S_CPOL member */
    (*I2S_InitStruct).I2S_CPOL = 0 as libc::c_int as uint16_t;
}
/* *
  * @brief  Enables or disables the specified SPI peripheral.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_Cmd(mut SPIx: *mut SPI_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI peripheral */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x40 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI peripheral */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xffbf as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified SPI peripheral (in I2S mode).
  * @param  SPIx: where x can be 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx peripheral. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn I2S_Cmd(mut SPIx: *mut SPI_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI peripheral (in I2S mode) */
        ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x400 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI peripheral (in I2S mode) */
        ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).I2SCFGR
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfbff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the specified SPI/I2S interrupts.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  *   - 2 or 3 in I2S mode
  * @param  SPI_I2S_IT: specifies the SPI/I2S interrupt source to be enabled or disabled. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_IT_TXE: Tx buffer empty interrupt mask
  *     @arg SPI_I2S_IT_RXNE: Rx buffer not empty interrupt mask
  *     @arg SPI_I2S_IT_ERR: Error interrupt mask
  * @param  NewState: new state of the specified SPI/I2S interrupt.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ITConfig(mut SPIx: *mut SPI_TypeDef,
                                          mut SPI_I2S_IT: uint8_t,
                                          mut NewState: FunctionalState) {
    let mut itpos: uint16_t = 0 as libc::c_int as uint16_t;
    let mut itmask: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Get the SPI/I2S IT index */
    itpos = (SPI_I2S_IT as libc::c_int >> 4 as libc::c_int) as uint16_t;
    /* Set the IT mask */
    itmask =
        ((1 as libc::c_int as uint16_t as libc::c_int) <<
             itpos as libc::c_int) as uint16_t;
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI/I2S interrupt */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         itmask as libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI/I2S interrupt */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(itmask as libc::c_int) as uint16_t
                                             as libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the SPIx/I2Sx DMA interface.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  *   - 2 or 3 in I2S mode
  * @param  SPI_I2S_DMAReq: specifies the SPI/I2S DMA transfer request to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg SPI_I2S_DMAReq_Tx: Tx buffer DMA transfer request
  *     @arg SPI_I2S_DMAReq_Rx: Rx buffer DMA transfer request
  * @param  NewState: new state of the selected SPI/I2S DMA transfer request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_DMACmd(mut SPIx: *mut SPI_TypeDef,
                                        mut SPI_I2S_DMAReq: uint16_t,
                                        mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI/I2S DMA requests */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         SPI_I2S_DMAReq as libc::c_int) as
                                        uint16_t as uint16_t)
    } else {
        /* Disable the selected SPI/I2S DMA requests */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(SPI_I2S_DMAReq as libc::c_int) as
                                             uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Transmits a Data through the SPIx/I2Sx peripheral.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  *   - 2 or 3 in I2S mode
  * @param  Data : Data to be transmitted.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_SendData(mut SPIx: *mut SPI_TypeDef,
                                          mut Data: uint16_t) {
    /* Check the parameters */
    /* Write in the DR register the data to be sent */
    ::core::ptr::write_volatile(&mut (*SPIx).DR as *mut uint16_t, Data);
}
/* *
  * @brief  Returns the most recent received data by the SPIx/I2Sx peripheral. 
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  *   - 2 or 3 in I2S mode
  * @retval The value of the received data.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ReceiveData(mut SPIx: *mut SPI_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Return the data in the DR register */
    return (*SPIx).DR;
}
/* *
  * @brief  Configures internally by software the NSS pin for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_NSSInternalSoft: specifies the SPI NSS internal state.
  *   This parameter can be one of the following values:
  *     @arg SPI_NSSInternalSoft_Set: Set NSS pin internally
  *     @arg SPI_NSSInternalSoft_Reset: Reset NSS pin internally
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_NSSInternalSoftwareConfig(mut SPIx:
                                                           *mut SPI_TypeDef,
                                                       mut SPI_NSSInternalSoft:
                                                           uint16_t) {
    /* Check the parameters */
    if SPI_NSSInternalSoft as libc::c_int !=
           0xfeff as libc::c_int as uint16_t as libc::c_int {
        /* Set NSS pin internally by software */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x100 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Reset NSS pin internally by software */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfeff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables the SS output for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx SS output. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_SSOutputCmd(mut SPIx: *mut SPI_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI SS output */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI SS output */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR2
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xfffb as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Configures the data size for the selected SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_DataSize: specifies the SPI data size.
  *   This parameter can be one of the following values:
  *     @arg SPI_DataSize_16b: Set data frame format to 16bit
  *     @arg SPI_DataSize_8b: Set data frame format to 8bit
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_DataSizeConfig(mut SPIx: *mut SPI_TypeDef,
                                            mut SPI_DataSize: uint16_t) {
    /* Check the parameters */
    /* Clear DFF bit */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x800 as libc::c_int as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Set new DFF bit value */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     SPI_DataSize as libc::c_int) as uint16_t
                                    as uint16_t);
}
/* *
  * @brief  Transmit the SPIx CRC value.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_TransmitCRC(mut SPIx: *mut SPI_TypeDef) {
    /* Check the parameters */
    /* Enable the selected SPI CRC transmission */
    ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x1000 as libc::c_int as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Enables or disables the CRC value calculation of the transferred bytes.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  NewState: new state of the SPIx CRC value calculation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_CalculateCRC(mut SPIx: *mut SPI_TypeDef,
                                          mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected SPI CRC calculation */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Disable the selected SPI CRC calculation */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xdfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Returns the transmit or the receive CRC register value for the specified SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_CRC: specifies the CRC register to be read.
  *   This parameter can be one of the following values:
  *     @arg SPI_CRC_Tx: Selects Tx CRC register
  *     @arg SPI_CRC_Rx: Selects Rx CRC register
  * @retval The selected CRC register value..
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_GetCRC(mut SPIx: *mut SPI_TypeDef,
                                    mut SPI_CRC: uint8_t) -> uint16_t {
    let mut crcreg: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    if SPI_CRC as libc::c_int != 0x1 as libc::c_int as uint8_t as libc::c_int
       {
        /* Get the Tx CRC register */
        crcreg = (*SPIx).TXCRCR
    } else {
        /* Get the Rx CRC register */
        crcreg = (*SPIx).RXCRCR
    }
    /* Return the selected CRC register */
    return crcreg;
}
/* *
  * @brief  Returns the CRC Polynomial register value for the specified SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @retval The CRC Polynomial register value.
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_GetCRCPolynomial(mut SPIx: *mut SPI_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Return the CRC polynomial register */
    return (*SPIx).CRCPR;
}
/* *
  * @brief  Selects the data transfer direction in bi-directional mode for the specified SPI.
  * @param  SPIx: where x can be 1, 2 or 3 to select the SPI peripheral.
  * @param  SPI_Direction: specifies the data transfer direction in bi-directional mode. 
  *   This parameter can be one of the following values:
  *     @arg SPI_Direction_Tx: Selects Tx transmission direction
  *     @arg SPI_Direction_Rx: Selects Rx receive direction
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_BiDirectionalLineConfig(mut SPIx:
                                                         *mut SPI_TypeDef,
                                                     mut SPI_Direction:
                                                         uint16_t) {
    /* Check the parameters */
    if SPI_Direction as libc::c_int ==
           0x4000 as libc::c_int as uint16_t as libc::c_int {
        /* Set the Tx only mode */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4000 as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        /* Set the Rx only mode */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*SPIx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         0xbfff as libc::c_int as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Checks whether the specified SPI/I2S flag is set or not.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  *   - 2 or 3 in I2S mode
  * @param  SPI_I2S_FLAG: specifies the SPI/I2S flag to check. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_FLAG_TXE: Transmit buffer empty flag.
  *     @arg SPI_I2S_FLAG_RXNE: Receive buffer not empty flag.
  *     @arg SPI_I2S_FLAG_BSY: Busy flag.
  *     @arg SPI_I2S_FLAG_OVR: Overrun flag.
  *     @arg SPI_FLAG_MODF: Mode Fault flag.
  *     @arg SPI_FLAG_CRCERR: CRC Error flag.
  *     @arg I2S_FLAG_UDR: Underrun Error flag.
  *     @arg I2S_FLAG_CHSIDE: Channel Side flag.
  * @retval The new state of SPI_I2S_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_GetFlagStatus(mut SPIx: *mut SPI_TypeDef,
                                               mut SPI_I2S_FLAG: uint16_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified SPI/I2S flag */
    if (*SPIx).SR as libc::c_int & SPI_I2S_FLAG as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int {
        /* SPI_I2S_FLAG is set */
        bitstatus = SET
    } else {
        /* SPI_I2S_FLAG is reset */
        bitstatus = RESET
    }
    /* Return the SPI_I2S_FLAG status */
    return bitstatus;
}
/* *
  * @brief  Clears the SPIx CRC Error (CRCERR) flag.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  * @param  SPI_I2S_FLAG: specifies the SPI flag to clear. 
  *   This function clears only CRCERR flag.
  * @note
  *   - OVR (OverRun error) flag is cleared by software sequence: a read 
  *     operation to SPI_DR register (SPI_I2S_ReceiveData()) followed by a read 
  *     operation to SPI_SR register (SPI_I2S_GetFlagStatus()).
  *   - UDR (UnderRun error) flag is cleared by a read operation to 
  *     SPI_SR register (SPI_I2S_GetFlagStatus()).
  *   - MODF (Mode Fault) flag is cleared by software sequence: a read/write 
  *     operation to SPI_SR register (SPI_I2S_GetFlagStatus()) followed by a 
  *     write operation to SPI_CR1 register (SPI_Cmd() to enable the SPI).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ClearFlag(mut SPIx: *mut SPI_TypeDef,
                                           mut SPI_I2S_FLAG: uint16_t) {
    /* Check the parameters */
    /* Clear the selected SPI CRC Error (CRCERR) flag */
    ::core::ptr::write_volatile(&mut (*SPIx).SR as *mut uint16_t,
                                !(SPI_I2S_FLAG as libc::c_int) as uint16_t);
}
/* *
  * @brief  Checks whether the specified SPI/I2S interrupt has occurred or not.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  *   - 2 or 3 in I2S mode
  * @param  SPI_I2S_IT: specifies the SPI/I2S interrupt source to check. 
  *   This parameter can be one of the following values:
  *     @arg SPI_I2S_IT_TXE: Transmit buffer empty interrupt.
  *     @arg SPI_I2S_IT_RXNE: Receive buffer not empty interrupt.
  *     @arg SPI_I2S_IT_OVR: Overrun interrupt.
  *     @arg SPI_IT_MODF: Mode Fault interrupt.
  *     @arg SPI_IT_CRCERR: CRC Error interrupt.
  *     @arg I2S_IT_UDR: Underrun Error interrupt.
  * @retval The new state of SPI_I2S_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_GetITStatus(mut SPIx: *mut SPI_TypeDef,
                                             mut SPI_I2S_IT: uint8_t)
 -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itpos: uint16_t = 0 as libc::c_int as uint16_t;
    let mut itmask: uint16_t = 0 as libc::c_int as uint16_t;
    let mut enablestatus: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Get the SPI/I2S IT index */
    itpos =
        ((0x1 as libc::c_int) <<
             (SPI_I2S_IT as libc::c_int & 0xf as libc::c_int)) as uint16_t;
    /* Get the SPI/I2S IT mask */
    itmask = (SPI_I2S_IT as libc::c_int >> 4 as libc::c_int) as uint16_t;
    /* Set the IT mask */
    itmask = ((0x1 as libc::c_int) << itmask as libc::c_int) as uint16_t;
    /* Get the SPI_I2S_IT enable bit status */
    enablestatus =
        ((*SPIx).CR2 as libc::c_int & itmask as libc::c_int) as uint16_t;
    /* Check the status of the specified SPI/I2S interrupt */
    if (*SPIx).SR as libc::c_int & itpos as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int &&
           enablestatus as libc::c_int != 0 {
        /* SPI_I2S_IT is set */
        bitstatus = SET
    } else {
        /* SPI_I2S_IT is reset */
        bitstatus = RESET
    }
    /* Return the SPI_I2S_IT status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_spi.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the SPI firmware 
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
/* * @addtogroup SPI
  * @{
  */
/* * @defgroup SPI_Exported_Types
  * @{
  */
/* * 
  * @brief  SPI Init structure definition  
  */
/* !< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_data_direction */
/* !< Specifies the SPI operating mode.
                                         This parameter can be a value of @ref SPI_mode */
/* !< Specifies the SPI data size.
                                         This parameter can be a value of @ref SPI_data_size */
/* !< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_Clock_Polarity */
/* !< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_Clock_Phase */
/* !< Specifies whether the NSS signal is managed by
                                         hardware (NSS pin) or by software using the SSI bit.
                                         This parameter can be a value of @ref SPI_Slave_Select_management */
/* !< Specifies the Baud Rate prescaler value which will be
                                         used to configure the transmit and receive SCK clock.
                                         This parameter can be a value of @ref SPI_BaudRate_Prescaler.
                                         @note The communication clock is derived from the master
                                               clock. The slave clock does not need to be set. */
/* !< Specifies whether data transfers start from MSB or LSB bit.
                                         This parameter can be a value of @ref SPI_MSB_LSB_transmission */
/* !< Specifies the polynomial used for the CRC calculation. */
/* * 
  * @brief  I2S Init structure definition  
  */
/* !< Specifies the I2S operating mode.
                                  This parameter can be a value of @ref I2S_Mode */
/* !< Specifies the standard used for the I2S communication.
                                  This parameter can be a value of @ref I2S_Standard */
/* !< Specifies the data format for the I2S communication.
                                  This parameter can be a value of @ref I2S_Data_Format */
/* !< Specifies whether the I2S MCLK output is enabled or not.
                                  This parameter can be a value of @ref I2S_MCLK_Output */
/* !< Specifies the frequency selected for the I2S communication.
                                  This parameter can be a value of @ref I2S_Audio_Frequency */
/* !< Specifies the idle state of the I2S clock.
                                  This parameter can be a value of @ref I2S_Clock_Polarity */
/* *
  * @}
  */
/* * @defgroup SPI_Exported_Constants
  * @{
  */
/* * @defgroup SPI_data_direction 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_data_size 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Clock_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Clock_Phase 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Slave_Select_management 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_BaudRate_Prescaler 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_MSB_LSB_transmission 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Standard 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Data_Format 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_MCLK_Output 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Audio_Frequency 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup I2S_Clock_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_I2S_DMA_transfer_requests 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_NSS_internal_software_management 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_Transmit_Receive 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_direction_transmit_receive 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_I2S_interrupts_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_I2S_flags_definition 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_CRC_polynomial 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup SPI_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup SPI_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the SPIx CRC Error (CRCERR) interrupt pending bit.
  * @param  SPIx: where x can be
  *   - 1, 2 or 3 in SPI mode 
  * @param  SPI_I2S_IT: specifies the SPI interrupt pending bit to clear.
  *   This function clears only CRCERR interrupt pending bit.   
  * @note
  *   - OVR (OverRun Error) interrupt pending bit is cleared by software 
  *     sequence: a read operation to SPI_DR register (SPI_I2S_ReceiveData()) 
  *     followed by a read operation to SPI_SR register (SPI_I2S_GetITStatus()).
  *   - UDR (UnderRun Error) interrupt pending bit is cleared by a read 
  *     operation to SPI_SR register (SPI_I2S_GetITStatus()).
  *   - MODF (Mode Fault) interrupt pending bit is cleared by software sequence:
  *     a read/write operation to SPI_SR register (SPI_I2S_GetITStatus()) 
  *     followed by a write operation to SPI_CR1 register (SPI_Cmd() to enable 
  *     the SPI).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn SPI_I2S_ClearITPendingBit(mut SPIx: *mut SPI_TypeDef,
                                                   mut SPI_I2S_IT: uint8_t) {
    let mut itpos: uint16_t = 0 as libc::c_int as uint16_t;
    /* Check the parameters */
    /* Get the SPI IT index */
    itpos =
        ((0x1 as libc::c_int) <<
             (SPI_I2S_IT as libc::c_int & 0xf as libc::c_int)) as uint16_t;
    /* Clear the selected SPI CRC Error (CRCERR) interrupt pending bit */
    ::core::ptr::write_volatile(&mut (*SPIx).SR as *mut uint16_t,
                                !(itpos as libc::c_int) as uint16_t);
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
