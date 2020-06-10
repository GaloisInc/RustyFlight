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
pub type u8_0 = uint8_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub CRL: uint32_t,
    pub CRH: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub BRR: uint32_t,
    pub LCKR: uint32_t,
}
/* * 
  * @brief Alternate Function I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct AFIO_TypeDef {
    pub EVCR: uint32_t,
    pub MAPR: uint32_t,
    pub EXTICR: [uint32_t; 4],
    pub RESERVED0: uint32_t,
    pub MAPR2: uint32_t,
}
pub type GPIOSpeed_TypeDef = libc::c_uint;
pub const GPIO_Speed_50MHz: GPIOSpeed_TypeDef = 3;
pub const GPIO_Speed_2MHz: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_10MHz: GPIOSpeed_TypeDef = 1;
pub type GPIOMode_TypeDef = libc::c_uint;
pub const GPIO_Mode_AF_PP: GPIOMode_TypeDef = 24;
pub const GPIO_Mode_AF_OD: GPIOMode_TypeDef = 28;
pub const GPIO_Mode_Out_PP: GPIOMode_TypeDef = 16;
pub const GPIO_Mode_Out_OD: GPIOMode_TypeDef = 20;
pub const GPIO_Mode_IPU: GPIOMode_TypeDef = 72;
pub const GPIO_Mode_IPD: GPIOMode_TypeDef = 40;
pub const GPIO_Mode_IN_FLOATING: GPIOMode_TypeDef = 4;
pub const GPIO_Mode_AIN: GPIOMode_TypeDef = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint16_t,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_Mode: GPIOMode_TypeDef,
}
pub type BitAction = libc::c_uint;
pub const Bit_SET: BitAction = 1;
pub const Bit_RESET: BitAction = 0;
/* *
  * @}
  */
/* * @defgroup GPIO_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_DeInit(mut GPIOx: *mut GPIO_TypeDef) {
    /* Check the parameters */
    if GPIOx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x800 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x4 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x4 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0xc00
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x8 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x8 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1000
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x10 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x10 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x20 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x20 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1800
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x40 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x40 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x1c00
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x80 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x80 as libc::c_int as uint32_t, DISABLE);
    } else if GPIOx ==
                  (0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x2000
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut GPIO_TypeDef {
        RCC_APB2PeriphResetCmd(0x100 as libc::c_int as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x100 as libc::c_int as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Deinitializes the Alternate Functions (remap, event control
  *   and EXTI configuration) registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_AFIODeInit() {
    RCC_APB2PeriphResetCmd(0x1 as libc::c_int as uint32_t, ENABLE);
    RCC_APB2PeriphResetCmd(0x1 as libc::c_int as uint32_t, DISABLE);
}
/* *
  * @brief  Initializes the GPIOx peripheral according to the specified
  *         parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that
  *         contains the configuration information for the specified GPIO peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_Init(mut GPIOx: *mut GPIO_TypeDef,
                                   mut GPIO_InitStruct:
                                       *mut GPIO_InitTypeDef) {
    let mut currentmode: uint32_t = 0 as libc::c_int as uint32_t;
    let mut currentpin: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pinpos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pos: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pinmask: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /*---------------------------- GPIO Mode Configuration -----------------------*/
    currentmode =
        (*GPIO_InitStruct).GPIO_Mode as uint32_t &
            0xf as libc::c_int as uint32_t;
    if (*GPIO_InitStruct).GPIO_Mode as uint32_t &
           0x10 as libc::c_int as uint32_t != 0 as libc::c_int as libc::c_uint
       {
        /* Check the parameters */
        /* Output mode */
        currentmode |= (*GPIO_InitStruct).GPIO_Speed as uint32_t
    }
    /*---------------------------- GPIO CRL Configuration ------------------------*/
  /* Configure the eight low port pins */
    if (*GPIO_InitStruct).GPIO_Pin as uint32_t &
           0xff as libc::c_int as uint32_t != 0 as libc::c_int as libc::c_uint
       {
        tmpreg = (*GPIOx).CRL;
        pinpos = 0 as libc::c_int as uint32_t;
        while pinpos < 0x8 as libc::c_int as libc::c_uint {
            pos = (0x1 as libc::c_int as uint32_t) << pinpos;
            /* Get the port pins position */
            currentpin = (*GPIO_InitStruct).GPIO_Pin as libc::c_uint & pos;
            if currentpin == pos {
                pos = pinpos << 2 as libc::c_int;
                /* Clear the corresponding low control register bits */
                pinmask = (0xf as libc::c_int as uint32_t) << pos;
                tmpreg &= !pinmask;
                /* Write the mode configuration in the corresponding bits */
                tmpreg |= currentmode << pos;
                /* Reset the corresponding ODR bit */
                if (*GPIO_InitStruct).GPIO_Mode as libc::c_uint ==
                       GPIO_Mode_IPD as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*GPIOx).BRR as
                                                    *mut uint32_t,
                                                (0x1 as libc::c_int as
                                                     uint32_t) << pinpos)
                } else if (*GPIO_InitStruct).GPIO_Mode as libc::c_uint ==
                              GPIO_Mode_IPU as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as
                                                    *mut uint32_t,
                                                (0x1 as libc::c_int as
                                                     uint32_t) << pinpos)
                }
            }
            pinpos = pinpos.wrapping_add(1)
        }
        ::core::ptr::write_volatile(&mut (*GPIOx).CRL as *mut uint32_t,
                                    tmpreg)
    }
    /* Set the corresponding ODR bit */
    /*---------------------------- GPIO CRH Configuration ------------------------*/
  /* Configure the eight high port pins */
    if (*GPIO_InitStruct).GPIO_Pin as libc::c_int > 0xff as libc::c_int {
        tmpreg = (*GPIOx).CRH;
        pinpos = 0 as libc::c_int as uint32_t;
        while pinpos < 0x8 as libc::c_int as libc::c_uint {
            pos =
                (0x1 as libc::c_int as uint32_t) <<
                    pinpos.wrapping_add(0x8 as libc::c_int as libc::c_uint);
            /* Get the port pins position */
            currentpin = (*GPIO_InitStruct).GPIO_Pin as libc::c_uint & pos;
            if currentpin == pos {
                pos = pinpos << 2 as libc::c_int;
                /* Clear the corresponding high control register bits */
                pinmask = (0xf as libc::c_int as uint32_t) << pos;
                tmpreg &= !pinmask;
                /* Write the mode configuration in the corresponding bits */
                tmpreg |= currentmode << pos;
                /* Reset the corresponding ODR bit */
                if (*GPIO_InitStruct).GPIO_Mode as libc::c_uint ==
                       GPIO_Mode_IPD as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*GPIOx).BRR as
                                                    *mut uint32_t,
                                                (0x1 as libc::c_int as
                                                     uint32_t) <<
                                                    pinpos.wrapping_add(0x8 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint))
                }
                /* Set the corresponding ODR bit */
                if (*GPIO_InitStruct).GPIO_Mode as libc::c_uint ==
                       GPIO_Mode_IPU as libc::c_int as libc::c_uint {
                    ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as
                                                    *mut uint32_t,
                                                (0x1 as libc::c_int as
                                                     uint32_t) <<
                                                    pinpos.wrapping_add(0x8 as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint))
                }
            }
            pinpos = pinpos.wrapping_add(1)
        }
        ::core::ptr::write_volatile(&mut (*GPIOx).CRH as *mut uint32_t,
                                    tmpreg)
    };
}
/* *
  * @brief  Fills each GPIO_InitStruct member with its default value.
  * @param  GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_StructInit(mut GPIO_InitStruct:
                                             *mut GPIO_InitTypeDef) {
    /* Reset GPIO init structure parameters values */
    (*GPIO_InitStruct).GPIO_Pin = 0xffff as libc::c_int as uint16_t;
    (*GPIO_InitStruct).GPIO_Speed = GPIO_Speed_2MHz;
    (*GPIO_InitStruct).GPIO_Mode = GPIO_Mode_IN_FLOATING;
}
/* *
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin:  specifies the port bit to read.
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The input port pin value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadInputDataBit(mut GPIOx: *mut GPIO_TypeDef,
                                               mut GPIO_Pin: uint16_t)
 -> uint8_t {
    let mut bitstatus: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    if (*GPIOx).IDR & GPIO_Pin as libc::c_uint !=
           Bit_RESET as libc::c_int as uint32_t {
        bitstatus = Bit_SET as libc::c_int as uint8_t
    } else { bitstatus = Bit_RESET as libc::c_int as uint8_t }
    return bitstatus;
}
/* *
  * @brief  Reads the specified GPIO input data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO input data port value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadInputData(mut GPIOx: *mut GPIO_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    return (*GPIOx).IDR as uint16_t;
}
/* *
  * @brief  Reads the specified output data port bit.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin:  specifies the port bit to read.
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The output port pin value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadOutputDataBit(mut GPIOx: *mut GPIO_TypeDef,
                                                mut GPIO_Pin: uint16_t)
 -> uint8_t {
    let mut bitstatus: uint8_t = 0 as libc::c_int as uint8_t;
    /* Check the parameters */
    if (*GPIOx).ODR & GPIO_Pin as libc::c_uint !=
           Bit_RESET as libc::c_int as uint32_t {
        bitstatus = Bit_SET as libc::c_int as uint8_t
    } else { bitstatus = Bit_RESET as libc::c_int as uint8_t }
    return bitstatus;
}
/* *
  * @brief  Reads the specified GPIO output data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ReadOutputData(mut GPIOx: *mut GPIO_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    return (*GPIOx).ODR as uint16_t;
}
/* *
  * @brief  Sets the selected data port bits.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
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
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ResetBits(mut GPIOx: *mut GPIO_TypeDef,
                                        mut GPIO_Pin: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*GPIOx).BRR as *mut uint32_t,
                                GPIO_Pin as uint32_t);
}
/* *
  * @brief  Sets or clears the selected data port bit.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *   This parameter can be one of GPIO_Pin_x where x can be (0..15).
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enum values:
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
        ::core::ptr::write_volatile(&mut (*GPIOx).BRR as *mut uint32_t,
                                    GPIO_Pin as uint32_t)
    };
}
/* *
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_Write(mut GPIOx: *mut GPIO_TypeDef,
                                    mut PortVal: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*GPIOx).ODR as *mut uint32_t,
                                PortVal as uint32_t);
}
/* *
  * @brief  Locks GPIO Pins configuration registers.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
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
    /* Read LCKK bit*/
    tmp = (*GPIOx).LCKR;
    /* Read LCKK bit*/
    tmp = (*GPIOx).LCKR;
}
/* *
  * @brief  Selects the GPIO pin used as Event output.
  * @param  GPIO_PortSource: selects the GPIO port to be used as source
  *   for Event output.
  *   This parameter can be GPIO_PortSourceGPIOx where x can be (A..E).
  * @param  GPIO_PinSource: specifies the pin for the Event output.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_EventOutputConfig(mut GPIO_PortSource: uint8_t,
                                                mut GPIO_PinSource: uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut AFIO_TypeDef)).EVCR;
    /* Clear the PORT[6:4] and PIN[3:0] bits */
    tmpreg &= 0xff80 as libc::c_int as uint16_t as libc::c_uint;
    tmpreg |= (GPIO_PortSource as uint32_t) << 0x4 as libc::c_int;
    tmpreg |= GPIO_PinSource as libc::c_uint;
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x10000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut AFIO_TypeDef)).EVCR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the Event Output.
  * @param  NewState: new state of the Event output.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_EventOutputCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x10000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_int
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x7
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     uint8_t
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4
                                                                                                                                                                                                                                                         as
                                                                                                                                                                                                                                                         libc::c_int)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIO_Remap: selects the pin to remap.
  *   This parameter can be one of the following values:
  *     @arg GPIO_Remap_SPI1             : SPI1 Alternate Function mapping
  *     @arg GPIO_Remap_I2C1             : I2C1 Alternate Function mapping
  *     @arg GPIO_Remap_USART1           : USART1 Alternate Function mapping
  *     @arg GPIO_Remap_USART2           : USART2 Alternate Function mapping
  *     @arg GPIO_PartialRemap_USART3    : USART3 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_USART3       : USART3 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap_TIM1      : TIM1 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM1         : TIM1 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap1_TIM2     : TIM2 Partial1 Alternate Function mapping
  *     @arg GPIO_PartialRemap2_TIM2     : TIM2 Partial2 Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM2         : TIM2 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap_TIM3      : TIM3 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM3         : TIM3 Full Alternate Function mapping
  *     @arg GPIO_Remap_TIM4             : TIM4 Alternate Function mapping
  *     @arg GPIO_Remap1_CAN1            : CAN1 Alternate Function mapping
  *     @arg GPIO_Remap2_CAN1            : CAN1 Alternate Function mapping
  *     @arg GPIO_Remap_PD01             : PD01 Alternate Function mapping
  *     @arg GPIO_Remap_TIM5CH4_LSI      : LSI connected to TIM5 Channel4 input capture for calibration
  *     @arg GPIO_Remap_ADC1_ETRGINJ     : ADC1 External Trigger Injected Conversion remapping
  *     @arg GPIO_Remap_ADC1_ETRGREG     : ADC1 External Trigger Regular Conversion remapping
  *     @arg GPIO_Remap_ADC2_ETRGINJ     : ADC2 External Trigger Injected Conversion remapping
  *     @arg GPIO_Remap_ADC2_ETRGREG     : ADC2 External Trigger Regular Conversion remapping
  *     @arg GPIO_Remap_ETH              : Ethernet remapping (only for Connectivity line devices)
  *     @arg GPIO_Remap_CAN2             : CAN2 remapping (only for Connectivity line devices)
  *     @arg GPIO_Remap_SWJ_NoJTRST      : Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST
  *     @arg GPIO_Remap_SWJ_JTAGDisable  : JTAG-DP Disabled and SW-DP Enabled
  *     @arg GPIO_Remap_SWJ_Disable      : Full SWJ Disabled (JTAG-DP + SW-DP)
  *     @arg GPIO_Remap_SPI3             : SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices)
  *                                        When the SPI3/I2S3 is remapped using this function, the SWJ is configured
  *                                        to Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST.   
  *     @arg GPIO_Remap_TIM2ITR1_PTP_SOF : Ethernet PTP output or USB OTG SOF (Start of Frame) connected
  *                                        to TIM2 Internal Trigger 1 for calibration (only for Connectivity line devices)
  *                                        If the GPIO_Remap_TIM2ITR1_PTP_SOF is enabled the TIM2 ITR1 is connected to 
  *                                        Ethernet PTP output. When Reset TIM2 ITR1 is connected to USB OTG SOF output.    
  *     @arg GPIO_Remap_PTP_PPS          : Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices)
  *     @arg GPIO_Remap_TIM15            : TIM15 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM16            : TIM16 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM17            : TIM17 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_CEC              : CEC Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM1_DMA         : TIM1 DMA requests mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM9             : TIM9 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM10            : TIM10 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM11            : TIM11 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM13            : TIM13 Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_TIM14            : TIM14 Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_FSMC_NADV        : FSMC_NADV Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_TIM67_DAC_DMA    : TIM6/TIM7 and DAC DMA requests remapping (only for High density Value line devices)
  *     @arg GPIO_Remap_TIM12            : TIM12 Alternate Function mapping (only for High density Value line devices)
  *     @arg GPIO_Remap_MISC             : Miscellaneous Remap (DMA2 Channel5 Position and DAC Trigger remapping, 
  *                                        only for High density Value line devices)     
  * @param  NewState: new state of the port pin remapping.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_PinRemapConfig(mut GPIO_Remap: uint32_t,
                                             mut NewState: FunctionalState) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmp1: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut tmpmask: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    if GPIO_Remap & 0x80000000 as libc::c_uint == 0x80000000 as libc::c_uint {
        tmpreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut AFIO_TypeDef)).MAPR2
    } else {
        tmpreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut AFIO_TypeDef)).MAPR
    }
    tmpmask =
        (GPIO_Remap & 0xf0000 as libc::c_int as uint32_t) >>
            0x10 as libc::c_int;
    tmp = GPIO_Remap & 0xffff as libc::c_int as uint16_t as libc::c_uint;
    if GPIO_Remap &
           (0x200000 as libc::c_int as uint32_t |
                0x100000 as libc::c_int as uint32_t) ==
           0x200000 as libc::c_int as uint32_t |
               0x100000 as libc::c_int as uint32_t {
        tmpreg &= 0xf0ffffff as libc::c_uint;
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x10000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut AFIO_TypeDef)).MAPR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         0xf0ffffff as libc::c_uint) as
                                        uint32_t as uint32_t)
    } else if GPIO_Remap & 0x100000 as libc::c_int as uint32_t ==
                  0x100000 as libc::c_int as uint32_t {
        tmp1 = (0x3 as libc::c_int as uint32_t) << tmpmask;
        tmpreg &= !tmp1;
        tmpreg |= !(0xf0ffffff as libc::c_uint)
    } else {
        tmpreg &=
            !(tmp <<
                  (GPIO_Remap >>
                       0x15 as
                           libc::c_int).wrapping_mul(0x10 as libc::c_int as
                                                         libc::c_uint));
        tmpreg |= !(0xf0ffffff as libc::c_uint)
    }
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        tmpreg |=
            tmp <<
                (GPIO_Remap >>
                     0x15 as
                         libc::c_int).wrapping_mul(0x10 as libc::c_int as
                                                       libc::c_uint)
    }
    if GPIO_Remap & 0x80000000 as libc::c_uint == 0x80000000 as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut AFIO_TypeDef)).MAPR2
                                        as *mut uint32_t, tmpreg)
    } else {
        ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x10000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut AFIO_TypeDef)).MAPR as
                                        *mut uint32_t, tmpreg)
    };
}
/* *
  * @brief  Selects the GPIO pin used as EXTI Line.
  * @param  GPIO_PortSource: selects the GPIO port to be used as source for EXTI lines.
  *   This parameter can be GPIO_PortSourceGPIOx where x can be (A..G).
  * @param  GPIO_PinSource: specifies the EXTI line to be configured.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_EXTILineConfig(mut GPIO_PortSource: uint8_t,
                                             mut GPIO_PinSource: uint8_t) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmp =
        (0xf as libc::c_int as uint32_t) <<
            0x4 as libc::c_int *
                (GPIO_PinSource as libc::c_int &
                     0x3 as libc::c_int as uint8_t as libc::c_int);
    let ref mut fresh1 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as
               *mut AFIO_TypeDef)).EXTICR[(GPIO_PinSource as libc::c_int >>
                                               0x2 as libc::c_int) as usize];
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmp) as uint32_t as
                                    uint32_t);
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x10000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as
               *mut AFIO_TypeDef)).EXTICR[(GPIO_PinSource as libc::c_int >>
                                               0x2 as libc::c_int) as usize];
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (GPIO_PortSource as uint32_t) <<
                                         0x4 as libc::c_int *
                                             (GPIO_PinSource as libc::c_int &
                                                  0x3 as libc::c_int as
                                                      uint8_t as libc::c_int))
                                    as uint32_t as uint32_t);
}
/* *
  ******************************************************************************
  * @file    stm32f10x_gpio.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the GPIO 
  *          firmware library.
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
/* * @addtogroup GPIO
  * @{
  */
/* * @defgroup GPIO_Exported_Types
  * @{
  */
/* * 
  * @brief  Output Maximum frequency selection  
  */
/* * 
  * @brief  Configuration Mode enumeration  
  */
/* * 
  * @brief  GPIO Init structure definition  
  */
/* !< Specifies the GPIO pins to be configured.
                                      This parameter can be any value of @ref GPIO_pins_define */
/* !< Specifies the speed for the selected pins.
                                      This parameter can be a value of @ref GPIOSpeed_TypeDef */
/* !< Specifies the operating mode for the selected pins.
                                      This parameter can be a value of @ref GPIOMode_TypeDef */
/* * 
  * @brief  Bit_SET and Bit_RESET enumeration  
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Exported_Constants
  * @{
  */
/* * @defgroup GPIO_pins_define 
  * @{
  */
/* !< Pin 0 selected */
/* !< Pin 1 selected */
/* !< Pin 2 selected */
/* !< Pin 3 selected */
/* !< Pin 4 selected */
/* !< Pin 5 selected */
/* !< Pin 6 selected */
/* !< Pin 7 selected */
/* !< Pin 8 selected */
/* !< Pin 9 selected */
/* !< Pin 10 selected */
/* !< Pin 11 selected */
/* !< Pin 12 selected */
/* !< Pin 13 selected */
/* !< Pin 14 selected */
/* !< Pin 15 selected */
/* !< All pins selected */
/* *
  * @}
  */
/* * @defgroup GPIO_Remap_define 
  * @{
  */
/* !< SPI1 Alternate Function mapping */
/* !< I2C1 Alternate Function mapping */
/* !< USART1 Alternate Function mapping */
/* !< USART2 Alternate Function mapping */
/* !< USART3 Partial Alternate Function mapping */
/* !< USART3 Full Alternate Function mapping */
/* !< TIM1 Partial Alternate Function mapping */
/* !< TIM1 Full Alternate Function mapping */
/* !< TIM2 Partial1 Alternate Function mapping */
/* !< TIM2 Partial2 Alternate Function mapping */
/* !< TIM2 Full Alternate Function mapping */
/* !< TIM3 Partial Alternate Function mapping */
/* !< TIM3 Full Alternate Function mapping */
/* !< TIM4 Alternate Function mapping */
/* !< CAN1 Alternate Function mapping */
/* !< CAN1 Alternate Function mapping */
/* !< PD01 Alternate Function mapping */
/* !< LSI connected to TIM5 Channel4 input capture for calibration */
/* !< ADC1 External Trigger Injected Conversion remapping */
/* !< ADC1 External Trigger Regular Conversion remapping */
/* !< ADC2 External Trigger Injected Conversion remapping */
/* !< ADC2 External Trigger Regular Conversion remapping */
/* !< Ethernet remapping (only for Connectivity line devices) */
/* !< CAN2 remapping (only for Connectivity line devices) */
/* !< Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */
/* !< JTAG-DP Disabled and SW-DP Enabled */
/* !< Full SWJ Disabled (JTAG-DP + SW-DP) */
/* !< SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices) */
/* !< Ethernet PTP output or USB OTG SOF (Start of Frame) connected
                                                                 to TIM2 Internal Trigger 1 for calibration
                                                                 (only for Connectivity line devices) */
/* !< Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices) */
/* !< TIM15 Alternate Function mapping (only for Value line devices) */
/* !< TIM16 Alternate Function mapping (only for Value line devices) */
/* !< TIM17 Alternate Function mapping (only for Value line devices) */
/* !< CEC Alternate Function mapping (only for Value line devices) */
/* !< TIM1 DMA requests mapping (only for Value line devices) */
/* !< TIM9 Alternate Function mapping (only for XL-density devices) */
/* !< TIM10 Alternate Function mapping (only for XL-density devices) */
/* !< TIM11 Alternate Function mapping (only for XL-density devices) */
/* !< TIM13 Alternate Function mapping (only for High density Value line and XL-density devices) */
/* !< TIM14 Alternate Function mapping (only for High density Value line and XL-density devices) */
/* !< FSMC_NADV Alternate Function mapping (only for High density Value line and XL-density devices) */
/* !< TIM6/TIM7 and DAC DMA requests remapping (only for High density Value line devices) */
/* !< TIM12 Alternate Function mapping (only for High density Value line devices) */
/* !< Miscellaneous Remap (DMA2 Channel5 Position and DAC Trigger remapping, 
                                                                 only for High density Value line devices) */
/* *
  * @}
  */
/* * @defgroup GPIO_Port_Sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Pin_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Ethernet_Media_Interface 
  * @{
  */
/* *
  * @}
  */                                                
/* *
  * @}
  */
/* * @defgroup GPIO_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup GPIO_Exported_Functions
  * @{
  */
/* *
  * @brief  Selects the Ethernet media interface.
  * @note   This function applies only to STM32 Connectivity line devices.  
  * @param  GPIO_ETH_MediaInterface: specifies the Media Interface mode.
  *   This parameter can be one of the following values:
  *     @arg GPIO_ETH_MediaInterface_MII: MII mode
  *     @arg GPIO_ETH_MediaInterface_RMII: RMII mode    
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn GPIO_ETH_MediaInterfaceConfig(mut GPIO_ETH_MediaInterface:
                                                           uint32_t) {
    /* Configure MII_RMII selection bit */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x10000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x4
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_int
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x17
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     u8_0
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4
                                                                                                                                                                                                                                                         as
                                                                                                                                                                                                                                                         libc::c_int)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t,
                                GPIO_ETH_MediaInterface);
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
