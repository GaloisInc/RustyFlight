use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub PLLCFGR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub AHB1RSTR: uint32_t,
    pub AHB2RSTR: uint32_t,
    pub AHB3RSTR: uint32_t,
    pub RESERVED0: uint32_t,
    pub APB1RSTR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub RESERVED1: [uint32_t; 2],
    pub AHB1ENR: uint32_t,
    pub AHB2ENR: uint32_t,
    pub AHB3ENR: uint32_t,
    pub RESERVED2: uint32_t,
    pub APB1ENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub RESERVED3: [uint32_t; 2],
    pub AHB1LPENR: uint32_t,
    pub AHB2LPENR: uint32_t,
    pub AHB3LPENR: uint32_t,
    pub RESERVED4: uint32_t,
    pub APB1LPENR: uint32_t,
    pub APB2LPENR: uint32_t,
    pub RESERVED5: [uint32_t; 2],
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
    pub RESERVED6: [uint32_t; 2],
    pub SSCGR: uint32_t,
    pub PLLI2SCFGR: uint32_t,
    pub PLLSAICFGR: uint32_t,
    pub DCKCFGR1: uint32_t,
    pub DCKCFGR2: uint32_t,
}
/* *
  * @brief TIM
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint32_t,
    pub ARR: uint32_t,
    pub RCR: uint32_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint32_t,
    pub DMAR: uint32_t,
    pub OR: uint32_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
}
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_InitTypeDef {
    pub Prescaler: uint16_t,
    pub CounterMode: uint32_t,
    pub Autoreload: uint32_t,
    pub ClockDivision: uint32_t,
    pub RepetitionCounter: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_OC_InitTypeDef {
    pub OCMode: uint32_t,
    pub OCState: uint32_t,
    pub OCNState: uint32_t,
    pub CompareValue: uint32_t,
    pub OCPolarity: uint32_t,
    pub OCNPolarity: uint32_t,
    pub OCIdleState: uint32_t,
    pub OCNIdleState: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_IC_InitTypeDef {
    pub ICPolarity: uint32_t,
    pub ICActiveInput: uint32_t,
    pub ICPrescaler: uint32_t,
    pub ICFilter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_ENCODER_InitTypeDef {
    pub EncoderMode: uint32_t,
    pub IC1Polarity: uint32_t,
    pub IC1ActiveInput: uint32_t,
    pub IC1Prescaler: uint32_t,
    pub IC1Filter: uint32_t,
    pub IC2Polarity: uint32_t,
    pub IC2ActiveInput: uint32_t,
    pub IC2Prescaler: uint32_t,
    pub IC2Filter: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_HALLSENSOR_InitTypeDef {
    pub IC1Polarity: uint32_t,
    pub IC1Prescaler: uint32_t,
    pub IC1Filter: uint32_t,
    pub CommutationDelay: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_TIM_BDTR_InitTypeDef {
    pub OSSRState: uint32_t,
    pub OSSIState: uint32_t,
    pub LockLevel: uint32_t,
    pub DeadTime: uint8_t,
    pub BreakState: uint16_t,
    pub BreakPolarity: uint32_t,
    pub BreakFilter: uint32_t,
    pub Break2State: uint32_t,
    pub Break2Polarity: uint32_t,
    pub Break2Filter: uint32_t,
    pub AutomaticOutput: uint32_t,
}
#[inline]
unsafe extern "C" fn LL_TIM_SetPrescaler(mut TIMx: *mut TIM_TypeDef,
                                         mut Prescaler: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).PSC as *mut uint32_t, Prescaler);
}
#[inline]
unsafe extern "C" fn LL_TIM_SetAutoReload(mut TIMx: *mut TIM_TypeDef,
                                          mut AutoReload: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).ARR as *mut uint32_t,
                                AutoReload);
}
#[inline]
unsafe extern "C" fn LL_TIM_SetRepetitionCounter(mut TIMx: *mut TIM_TypeDef,
                                                 mut RepetitionCounter:
                                                     uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).RCR as *mut uint32_t,
                                RepetitionCounter);
}
#[inline]
unsafe extern "C" fn LL_TIM_OC_SetCompareCH1(mut TIMx: *mut TIM_TypeDef,
                                             mut CompareValue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCR1 as *mut uint32_t,
                                CompareValue);
}
#[inline]
unsafe extern "C" fn LL_TIM_OC_SetCompareCH2(mut TIMx: *mut TIM_TypeDef,
                                             mut CompareValue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCR2 as *mut uint32_t,
                                CompareValue);
}
#[inline]
unsafe extern "C" fn LL_TIM_OC_SetCompareCH3(mut TIMx: *mut TIM_TypeDef,
                                             mut CompareValue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCR3 as *mut uint32_t,
                                CompareValue);
}
#[inline]
unsafe extern "C" fn LL_TIM_OC_SetCompareCH4(mut TIMx: *mut TIM_TypeDef,
                                             mut CompareValue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCR4 as *mut uint32_t,
                                CompareValue);
}
#[inline]
unsafe extern "C" fn LL_TIM_OC_SetCompareCH5(mut TIMx: *mut TIM_TypeDef,
                                             mut CompareValue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                CompareValue);
}
#[inline]
unsafe extern "C" fn LL_TIM_OC_SetCompareCH6(mut TIMx: *mut TIM_TypeDef,
                                             mut CompareValue: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).CCR6 as *mut uint32_t,
                                CompareValue);
}
#[inline]
unsafe extern "C" fn LL_TIM_SetEncoderMode(mut TIMx: *mut TIM_TypeDef,
                                           mut EncoderMode: uint32_t) {
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (*TIMx).SMCR &
                                    !((0x10007 as libc::c_uint) <<
                                          0 as libc::c_uint) | EncoderMode);
}
#[inline]
unsafe extern "C" fn LL_TIM_GenerateEvent_UPDATE(mut TIMx: *mut TIM_TypeDef) {
    ::core::ptr::write_volatile(&mut (*TIMx).EGR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).EGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Force APB1 peripherals reset.
  * @rmtoll APB1RSTR     TIM2RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM3RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM4RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM5RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM6RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM7RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM12RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM13RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     TIM14RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     LPTIM1RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     WWDGRST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     SPI2RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     SPI3RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     SPDIFRXRST     LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     USART2RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     USART3RST      LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     UART4RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     UART5RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     I2C1RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     I2C2RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     I2C3RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     I2C4RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CAN1RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CAN2RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CAN3RST        LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     CECRST         LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     PWRRST         LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     DACRST         LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     UART7RST       LL_APB1_GRP1_ForceReset\n
  *         APB1RSTR     UART8RST       LL_APB1_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3 
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPDIFRX (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C3
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC  (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART8
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_APB1_GRP1_ForceReset(mut Periphs: uint32_t) {
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1RSTR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Periphs) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Release APB1 peripherals reset.
  * @rmtoll APB1RSTR     TIM2RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM3RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM4RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM5RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM6RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM7RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM12RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM13RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     TIM14RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     LPTIM1RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     WWDGRST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     SPI2RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     SPI3RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     SPDIFRXRST     LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     USART2RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     USART3RST      LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     UART4RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     UART5RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     I2C1RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     I2C2RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     I2C3RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     I2C4RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CAN1RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CAN2RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CAN3RST        LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     CECRST         LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     PWRRST         LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     DACRST         LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     UART7RST       LL_APB1_GRP1_ReleaseReset\n
  *         APB1RSTR     UART8RST       LL_APB1_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM2
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM3 
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM4
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM5 
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM6
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM7
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM12
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM13
  *         @arg @ref LL_APB1_GRP1_PERIPH_TIM14
  *         @arg @ref LL_APB1_GRP1_PERIPH_LPTIM1
  *         @arg @ref LL_APB1_GRP1_PERIPH_WWDG
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI2 
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPI3
  *         @arg @ref LL_APB1_GRP1_PERIPH_SPDIFRX (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART2
  *         @arg @ref LL_APB1_GRP1_PERIPH_USART3
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART4
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART5
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C1
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C2
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C3
  *         @arg @ref LL_APB1_GRP1_PERIPH_I2C4 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN1
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN2 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CAN3 (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_CEC  (*)
  *         @arg @ref LL_APB1_GRP1_PERIPH_PWR
  *         @arg @ref LL_APB1_GRP1_PERIPH_DAC1
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART7
  *         @arg @ref LL_APB1_GRP1_PERIPH_UART8
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_APB1_GRP1_ReleaseReset(mut Periphs: uint32_t) {
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB1RSTR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !Periphs) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Force APB2 peripherals reset.
  * @rmtoll APB2RSTR     TIM1RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM8RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     USART1RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     USART6RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     ADCRST         LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SDMMC1RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SDMMC2RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SPI1RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SPI4RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SYSCFGRST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM9RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM10RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     TIM11RST       LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SPI5RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SPI6RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SAI1RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     SAI2RST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     LTDCRST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     DSIRST         LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     DFSDM1RST      LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     MDIORST        LL_APB2_GRP1_ForceReset\n
  *         APB2RSTR     OTGPHYCRST     LL_APB2_GRP1_ForceReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART6
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC
  *         @arg @ref LL_APB2_GRP1_PERIPH_SDMMC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SDMMC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI4
  *         @arg @ref LL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI5
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI6 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SAI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SAI2
  *         @arg @ref LL_APB2_GRP1_PERIPH_LTDC (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_DSI  (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_DFSDM1 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_MDIO (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_OTGPHYC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_APB2_GRP1_ForceReset(mut Periphs: uint32_t) {
    let ref mut fresh2 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2RSTR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | Periphs) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Release APB2 peripherals reset.
  * @rmtoll APB2RSTR     TIM1RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM8RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     USART1RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     USART6RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     ADCRST         LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SDMMC1RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SDMMC2RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SPI1RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SPI4RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SYSCFGRST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM9RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM10RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     TIM11RST       LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SPI5RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SPI6RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SAI1RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     SAI2RST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     LTDCRST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     DSIRST         LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     DFSDM1RST      LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     MDIORST        LL_APB2_GRP1_ReleaseReset\n
  *         APB2RSTR     OTGPHYCRST     LL_APB2_GRP1_ReleaseReset
  * @param  Periphs This parameter can be a combination of the following values:
  *         @arg @ref LL_APB2_GRP1_PERIPH_ALL
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM1
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM8
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART1
  *         @arg @ref LL_APB2_GRP1_PERIPH_USART6
  *         @arg @ref LL_APB2_GRP1_PERIPH_ADC
  *         @arg @ref LL_APB2_GRP1_PERIPH_SDMMC1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SDMMC2 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI4
  *         @arg @ref LL_APB2_GRP1_PERIPH_SYSCFG
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM9
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM10
  *         @arg @ref LL_APB2_GRP1_PERIPH_TIM11
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI5
  *         @arg @ref LL_APB2_GRP1_PERIPH_SPI6 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_SAI1
  *         @arg @ref LL_APB2_GRP1_PERIPH_SAI2
  *         @arg @ref LL_APB2_GRP1_PERIPH_LTDC (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_DSI  (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_DFSDM1 (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_MDIO (*)
  *         @arg @ref LL_APB2_GRP1_PERIPH_OTGPHYC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
*/
#[inline]
unsafe extern "C" fn LL_APB2_GRP1_ReleaseReset(mut Periphs: uint32_t) {
    let ref mut fresh3 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).APB2RSTR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !Periphs) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Disable commutation DMA request (COMDE).
  * @rmtoll DIER         COMDE         LL_TIM_DisableDMAReq_COM
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the commutation DMA request (COMDE) is enabled.
  * @rmtoll DIER         COMDE         LL_TIM_IsEnabledDMAReq_COM
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable trigger interrupt (TDE).
  * @rmtoll DIER         TDE           LL_TIM_EnableDMAReq_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable trigger interrupt (TDE).
  * @rmtoll DIER         TDE           LL_TIM_DisableDMAReq_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the trigger interrupt (TDE) is enabled.
  * @rmtoll DIER         TDE           LL_TIM_IsEnabledDMAReq_TRIG
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_EVENT_Management EVENT-Management
  * @{
  */
/* *
  * @brief  Generate an update event.
  * @rmtoll EGR          UG            LL_TIM_GenerateEvent_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate Capture/Compare 1 event.
  * @rmtoll EGR          CC1G          LL_TIM_GenerateEvent_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate Capture/Compare 2 event.
  * @rmtoll EGR          CC2G          LL_TIM_GenerateEvent_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate Capture/Compare 3 event.
  * @rmtoll EGR          CC3G          LL_TIM_GenerateEvent_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate Capture/Compare 4 event.
  * @rmtoll EGR          CC4G          LL_TIM_GenerateEvent_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate commutation event.
  * @rmtoll EGR          COMG          LL_TIM_GenerateEvent_COM
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate trigger event.
  * @rmtoll EGR          TG            LL_TIM_GenerateEvent_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate break event.
  * @rmtoll EGR          BG            LL_TIM_GenerateEvent_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Generate break 2 event.
  * @rmtoll EGR          B2G           LL_TIM_GenerateEvent_BRK2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Init Initialisation and deinitialisation functions
  * @{
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup TIM_LL_Exported_Functions
  * @{
  */
/* * @addtogroup TIM_LL_EF_Init
  * @{
  */
/* *
  * @brief  Set TIMx registers to their reset values.
  * @param  TIMx Timer instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: invalid TIMx instance
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_DeInit(mut TIMx: *mut TIM_TypeDef)
 -> ErrorStatus {
    let mut result: ErrorStatus = SUCCESS;
    /* Check the parameters */
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef {
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 0 as libc::c_uint);
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) << 0 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint)
                      as *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 0 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 0 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 1 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 1 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 2 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 2 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 3 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 3 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x1000 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 4 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 4 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x1400 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 5 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 5 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef {
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 1 as libc::c_uint);
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) << 1 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x4000
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef {
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 16 as libc::c_uint);
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      16 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x4400
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef {
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 17 as libc::c_uint);
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      17 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x10000 as
                                                      libc::c_uint).wrapping_add(0x4800
                                                                                     as
                                                                                     libc::c_uint)
                      as *mut TIM_TypeDef {
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 18 as libc::c_uint);
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      18 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x1800 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 6 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 6 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x1c00 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 7 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 7 as libc::c_uint);
    } else if TIMx ==
                  (0x40000000 as
                       libc::c_uint).wrapping_add(0x2000 as libc::c_uint) as
                      *mut TIM_TypeDef {
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 8 as libc::c_uint);
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) << 8 as libc::c_uint);
    } else { result = ERROR }
    return result;
}
/* *
  * @brief  Set the fields of the time base unit configuration data structure
  *         to their default values.
  * @param  TIM_InitStruct pointer to a @ref LL_TIM_InitTypeDef structure (time base unit configuration data structure)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_StructInit(mut TIM_InitStruct:
                                               *mut LL_TIM_InitTypeDef) {
    /* Set the default configuration */
    (*TIM_InitStruct).Prescaler = 0 as libc::c_uint as uint16_t;
    (*TIM_InitStruct).CounterMode = 0 as libc::c_uint;
    (*TIM_InitStruct).Autoreload = 0xffffffff as libc::c_uint;
    (*TIM_InitStruct).ClockDivision = 0 as libc::c_uint;
    (*TIM_InitStruct).RepetitionCounter = 0 as libc::c_uint as uint8_t;
}
/* *
  * @brief  Configure the TIMx time base unit.
  * @param  TIMx Timer Instance
  * @param  TIM_InitStruct pointer to a @ref LL_TIM_InitTypeDef structure (TIMx time base unit configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_InitStruct:
                                         *mut LL_TIM_InitTypeDef)
 -> ErrorStatus {
    let mut tmpcr1: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    tmpcr1 = (*TIMx).CR1;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Select the Counter Mode */
        tmpcr1 =
            tmpcr1 &
                !((0x1 as libc::c_uint) << 4 as libc::c_uint |
                      (0x3 as libc::c_uint) << 5 as libc::c_uint) |
                (*TIM_InitStruct).CounterMode
    }
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as libc::c_uint).wrapping_add(0 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x400 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x800 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0xc00 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x4000
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x4400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x4800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x1800 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x1c00 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x2000 as libc::c_uint) as
                   *mut TIM_TypeDef {
        /* Set the clock division */
        tmpcr1 =
            tmpcr1 & !((0x3 as libc::c_uint) << 8 as libc::c_uint) |
                (*TIM_InitStruct).ClockDivision
    }
    /* Write to TIMx CR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint32_t, tmpcr1);
    /* Set the Autoreload value */
    LL_TIM_SetAutoReload(TIMx, (*TIM_InitStruct).Autoreload);
    /* Set the Prescaler value */
    LL_TIM_SetPrescaler(TIMx, (*TIM_InitStruct).Prescaler as uint32_t);
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the Repetition Counter value */
        LL_TIM_SetRepetitionCounter(TIMx,
                                    (*TIM_InitStruct).RepetitionCounter as
                                        uint32_t);
    }
    /* Generate an update event to reload the Prescaler
     and the repetition counter value (if applicable) immediately */
    LL_TIM_GenerateEvent_UPDATE(TIMx);
    return SUCCESS;
}
/* *
  * @brief  Set the fields of the TIMx output channel configuration data
  *         structure to their default values.
  * @param  TIM_OC_InitStruct pointer to a @ref LL_TIM_OC_InitTypeDef structure (the output channel configuration data structure)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_OC_StructInit(mut TIM_OC_InitStruct:
                                                  *mut LL_TIM_OC_InitTypeDef) {
    /* Set the default configuration */
    (*TIM_OC_InitStruct).OCMode = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).OCState = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).OCNState = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).CompareValue = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).OCPolarity = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).OCNPolarity = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).OCIdleState = 0 as libc::c_uint;
    (*TIM_OC_InitStruct).OCNIdleState = 0 as libc::c_uint;
}
/* *
  * @brief  Configure the TIMx output channel.
  * @param  TIMx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  TIM_OC_InitStruct pointer to a @ref LL_TIM_OC_InitTypeDef structure (TIMx output channel configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx output channel is initialized
  *          - ERROR: TIMx output channel is not initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_OC_Init(mut TIMx: *mut TIM_TypeDef,
                                        mut Channel: uint32_t,
                                        mut TIM_OC_InitStruct:
                                            *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut result: ErrorStatus = ERROR;
    match Channel {
        1 => { result = OC1Config(TIMx, TIM_OC_InitStruct) }
        16 => { result = OC2Config(TIMx, TIM_OC_InitStruct) }
        256 => { result = OC3Config(TIMx, TIM_OC_InitStruct) }
        4096 => { result = OC4Config(TIMx, TIM_OC_InitStruct) }
        65536 => { result = OC5Config(TIMx, TIM_OC_InitStruct) }
        1048576 => { result = OC6Config(TIMx, TIM_OC_InitStruct) }
        _ => { }
    }
    return result;
}
/* *
  * @brief  Set the fields of the TIMx input channel configuration data
  *         structure to their default values.
  * @param  TIM_ICInitStruct pointer to a @ref LL_TIM_IC_InitTypeDef structure (the input channel configuration data structure)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_IC_StructInit(mut TIM_ICInitStruct:
                                                  *mut LL_TIM_IC_InitTypeDef) {
    /* Set the default configuration */
    (*TIM_ICInitStruct).ICPolarity = 0 as libc::c_uint;
    (*TIM_ICInitStruct).ICActiveInput =
        ((0x1 as libc::c_uint) << 0 as libc::c_uint) << 16 as libc::c_uint;
    (*TIM_ICInitStruct).ICPrescaler = 0 as libc::c_uint;
    (*TIM_ICInitStruct).ICFilter = 0 as libc::c_uint;
}
/* *
  * @brief  Configure the TIMx input channel.
  * @param  TIMx Timer Instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  TIM_IC_InitStruct pointer to a @ref LL_TIM_IC_InitTypeDef structure (TIMx input channel configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx output channel is initialized
  *          - ERROR: TIMx output channel is not initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_IC_Init(mut TIMx: *mut TIM_TypeDef,
                                        mut Channel: uint32_t,
                                        mut TIM_IC_InitStruct:
                                            *mut LL_TIM_IC_InitTypeDef)
 -> ErrorStatus {
    let mut result: ErrorStatus = ERROR;
    match Channel {
        1 => { result = IC1Config(TIMx, TIM_IC_InitStruct) }
        16 => { result = IC2Config(TIMx, TIM_IC_InitStruct) }
        256 => { result = IC3Config(TIMx, TIM_IC_InitStruct) }
        4096 => { result = IC4Config(TIMx, TIM_IC_InitStruct) }
        _ => { }
    }
    return result;
}
/* *
  * @brief  Fills each TIM_EncoderInitStruct field with its default value
  * @param  TIM_EncoderInitStruct pointer to a @ref LL_TIM_ENCODER_InitTypeDef structure (encoder interface configuration data structure)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_ENCODER_StructInit(mut TIM_EncoderInitStruct:
                                                       *mut LL_TIM_ENCODER_InitTypeDef) {
    /* Set the default configuration */
    (*TIM_EncoderInitStruct).EncoderMode =
        (0x1 as libc::c_uint) << 0 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC1Polarity = 0 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC1ActiveInput =
        ((0x1 as libc::c_uint) << 0 as libc::c_uint) << 16 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC1Prescaler = 0 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC1Filter = 0 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC2Polarity = 0 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC2ActiveInput =
        ((0x1 as libc::c_uint) << 0 as libc::c_uint) << 16 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC2Prescaler = 0 as libc::c_uint;
    (*TIM_EncoderInitStruct).IC2Filter = 0 as libc::c_uint;
}
/* *
  * @brief  Configure the encoder interface of the timer instance.
  * @param  TIMx Timer Instance
  * @param  TIM_EncoderInitStruct pointer to a @ref LL_TIM_ENCODER_InitTypeDef structure (TIMx encoder interface configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_ENCODER_Init(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_EncoderInitStruct:
                                                 *mut LL_TIM_ENCODER_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr1: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the CC1 and CC2: Reset the CC1E and CC2E Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               4 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Configure TI1 */
    tmpccmr1 &=
        !((0x3 as libc::c_uint) << 0 as libc::c_uint |
              (0xf as libc::c_uint) << 4 as libc::c_uint |
              (0x3 as libc::c_uint) << 2 as libc::c_uint);
    tmpccmr1 |= (*TIM_EncoderInitStruct).IC1ActiveInput >> 16 as libc::c_uint;
    tmpccmr1 |= (*TIM_EncoderInitStruct).IC1Filter >> 16 as libc::c_uint;
    tmpccmr1 |= (*TIM_EncoderInitStruct).IC1Prescaler >> 16 as libc::c_uint;
    /* Configure TI2 */
    tmpccmr1 &=
        !((0x3 as libc::c_uint) << 8 as libc::c_uint |
              (0xf as libc::c_uint) << 12 as libc::c_uint |
              (0x3 as libc::c_uint) << 10 as libc::c_uint);
    tmpccmr1 |= (*TIM_EncoderInitStruct).IC2ActiveInput >> 8 as libc::c_uint;
    tmpccmr1 |= (*TIM_EncoderInitStruct).IC2Filter >> 8 as libc::c_uint;
    tmpccmr1 |= (*TIM_EncoderInitStruct).IC2Prescaler >> 8 as libc::c_uint;
    /* Set TI1 and TI2 polarity and enable TI1 and TI2 */
    tmpccer &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 3 as libc::c_uint |
              (0x1 as libc::c_uint) << 5 as libc::c_uint |
              (0x1 as libc::c_uint) << 7 as libc::c_uint);
    tmpccer |= (*TIM_EncoderInitStruct).IC1Polarity;
    tmpccer |= (*TIM_EncoderInitStruct).IC2Polarity << 4 as libc::c_uint;
    tmpccer |=
        (0x1 as libc::c_uint) << 0 as libc::c_uint |
            (0x1 as libc::c_uint) << 4 as libc::c_uint;
    /* Set encoder mode */
    LL_TIM_SetEncoderMode(TIMx, (*TIM_EncoderInitStruct).EncoderMode);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Set the fields of the TIMx Hall sensor interface configuration data
  *         structure to their default values.
  * @param  TIM_HallSensorInitStruct pointer to a @ref LL_TIM_HALLSENSOR_InitTypeDef structure (HALL sensor interface configuration data structure)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_HALLSENSOR_StructInit(mut TIM_HallSensorInitStruct:
                                                          *mut LL_TIM_HALLSENSOR_InitTypeDef) {
    /* Set the default configuration */
    (*TIM_HallSensorInitStruct).IC1Polarity = 0 as libc::c_uint;
    (*TIM_HallSensorInitStruct).IC1Prescaler = 0 as libc::c_uint;
    (*TIM_HallSensorInitStruct).IC1Filter = 0 as libc::c_uint;
    (*TIM_HallSensorInitStruct).CommutationDelay = 0 as libc::c_uint;
}
/* *
  * @brief  Configure the Hall sensor interface of the timer instance.
  * @note TIMx CH1, CH2 and CH3 inputs connected through a XOR
  *       to the TI1 input channel
  * @note TIMx slave mode controller is configured in reset mode.
          Selected internal trigger is TI1F_ED.
  * @note Channel 1 is configured as input, IC1 is mapped on TRC.
  * @note Captured value stored in TIMx_CCR1 correspond to the time elapsed
  *       between 2 changes on the inputs. It gives information about motor speed.
  * @note Channel 2 is configured in output PWM 2 mode.
  * @note Compare value stored in TIMx_CCR2 corresponds to the commutation delay.
  * @note OC2REF is selected as trigger output on TRGO.
  * @note LL_TIM_IC_POLARITY_BOTHEDGE must not be used for TI1 when it is used
  *       when TIMx operates in Hall sensor interface mode.
  * @param  TIMx Timer Instance
  * @param  TIM_HallSensorInitStruct pointer to a @ref LL_TIM_HALLSENSOR_InitTypeDef structure (TIMx HALL sensor interface configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_HALLSENSOR_Init(mut TIMx: *mut TIM_TypeDef,
                                                mut TIM_HallSensorInitStruct:
                                                    *mut LL_TIM_HALLSENSOR_InitTypeDef)
 -> ErrorStatus {
    let mut tmpcr2: uint32_t = 0 as libc::c_uint;
    let mut tmpccmr1: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    let mut tmpsmcr: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the CC1 and CC2: Reset the CC1E and CC2E Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint |
                                           (0x1 as libc::c_uint) <<
                                               4 as libc::c_uint)) as uint32_t
                                    as uint32_t);
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*TIMx).SMCR;
    /* Connect TIMx_CH1, CH2 and CH3 pins to the TI1 input */
    tmpcr2 |= (0x1 as libc::c_uint) << 7 as libc::c_uint;
    /* OC2REF signal is used as trigger output (TRGO) */
    tmpcr2 |=
        (0x4 as libc::c_uint) << 4 as libc::c_uint |
            (0x1 as libc::c_uint) << 4 as libc::c_uint;
    /* Configure the slave mode controller */
    tmpsmcr &=
        !((0x7 as libc::c_uint) << 4 as libc::c_uint |
              (0x10007 as libc::c_uint) << 0 as libc::c_uint);
    tmpsmcr |= (0x4 as libc::c_uint) << 4 as libc::c_uint;
    tmpsmcr |= (0x4 as libc::c_uint) << 0 as libc::c_uint;
    /* Configure input channel 1 */
    tmpccmr1 &=
        !((0x3 as libc::c_uint) << 0 as libc::c_uint |
              (0xf as libc::c_uint) << 4 as libc::c_uint |
              (0x3 as libc::c_uint) << 2 as libc::c_uint);
    tmpccmr1 |=
        ((0x3 as libc::c_uint) << 0 as libc::c_uint) << 16 as libc::c_uint >>
            16 as libc::c_uint;
    tmpccmr1 |= (*TIM_HallSensorInitStruct).IC1Filter >> 16 as libc::c_uint;
    tmpccmr1 |=
        (*TIM_HallSensorInitStruct).IC1Prescaler >> 16 as libc::c_uint;
    /* Configure input channel 2 */
    tmpccmr1 &=
        !((0x1007 as libc::c_uint) << 12 as libc::c_uint |
              (0x1 as libc::c_uint) << 10 as libc::c_uint |
              (0x1 as libc::c_uint) << 11 as libc::c_uint |
              (0x1 as libc::c_uint) << 15 as libc::c_uint);
    tmpccmr1 |=
        ((0x4 as libc::c_uint) << 4 as libc::c_uint |
             (0x2 as libc::c_uint) << 4 as libc::c_uint |
             (0x1 as libc::c_uint) << 4 as libc::c_uint) << 8 as libc::c_uint;
    /* Set Channel 1 polarity and enable Channel 1 and Channel2 */
    tmpccer &=
        !((0x1 as libc::c_uint) << 1 as libc::c_uint |
              (0x1 as libc::c_uint) << 3 as libc::c_uint |
              (0x1 as libc::c_uint) << 5 as libc::c_uint |
              (0x1 as libc::c_uint) << 7 as libc::c_uint);
    tmpccer |= (*TIM_HallSensorInitStruct).IC1Polarity;
    tmpccer |=
        (0x1 as libc::c_uint) << 0 as libc::c_uint |
            (0x1 as libc::c_uint) << 4 as libc::c_uint;
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t, tmpsmcr);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    /* Write to TIMx CCR2 */
    LL_TIM_OC_SetCompareCH2(TIMx,
                            (*TIM_HallSensorInitStruct).CommutationDelay);
    return SUCCESS;
}
/* *
  * @brief  Set the fields of the Break and Dead Time configuration data structure
  *         to their default values.
  * @param  TIM_BDTRInitStruct pointer to a @ref LL_TIM_BDTR_InitTypeDef structure (Break and Dead Time configuration data structure)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_BDTR_StructInit(mut TIM_BDTRInitStruct:
                                                    *mut LL_TIM_BDTR_InitTypeDef) {
    /* Set the default configuration */
    (*TIM_BDTRInitStruct).OSSRState = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).OSSIState = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).LockLevel = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).DeadTime = 0 as libc::c_uint as uint8_t;
    (*TIM_BDTRInitStruct).BreakState = 0 as libc::c_uint as uint16_t;
    (*TIM_BDTRInitStruct).BreakPolarity = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).BreakFilter = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).Break2State = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).Break2Polarity = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).Break2Filter = 0 as libc::c_uint;
    (*TIM_BDTRInitStruct).AutomaticOutput = 0 as libc::c_uint;
}
/* *
  * @brief  Configure the Break and Dead Time feature of the timer instance.
  * @note As the bits BK2P, BK2E, BK2F[3:0], BKF[3:0], AOE, BKP, BKE, OSSI, OSSR
  *  and DTG[7:0] can be write-locked depending on the LOCK configuration, it 
  *  can be necessary to configure all of them during the first write access to
  *  the TIMx_BDTR register.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @note Macro @ref IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a second break input.
  * @param  TIMx Timer Instance
  * @param  TIM_BDTRInitStruct pointer to a @ref LL_TIM_BDTR_InitTypeDef structure (Break and Dead Time configuration data structure)
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Break and Dead Time is initialized
  *          - ERROR: not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_TIM_BDTR_Init(mut TIMx: *mut TIM_TypeDef,
                                          mut TIM_BDTRInitStruct:
                                              *mut LL_TIM_BDTR_InitTypeDef)
 -> ErrorStatus {
    let mut tmpbdtr: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
  the OSSI State, the dead time value and the Automatic Output Enable Bit */
    /* Set the BDTR bits */
    tmpbdtr =
        tmpbdtr & !((0xff as libc::c_uint) << 0 as libc::c_uint) |
            (*TIM_BDTRInitStruct).DeadTime as libc::c_uint;
    tmpbdtr =
        tmpbdtr & !((0x3 as libc::c_uint) << 8 as libc::c_uint) |
            (*TIM_BDTRInitStruct).LockLevel;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 10 as libc::c_uint) |
            (*TIM_BDTRInitStruct).OSSIState;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 11 as libc::c_uint) |
            (*TIM_BDTRInitStruct).OSSRState;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 12 as libc::c_uint) |
            (*TIM_BDTRInitStruct).BreakState as libc::c_uint;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 13 as libc::c_uint) |
            (*TIM_BDTRInitStruct).BreakPolarity;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 14 as libc::c_uint) |
            (*TIM_BDTRInitStruct).AutomaticOutput;
    tmpbdtr =
        tmpbdtr & !((0x1 as libc::c_uint) << 15 as libc::c_uint) |
            (*TIM_BDTRInitStruct).AutomaticOutput;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        tmpbdtr =
            tmpbdtr & !((0xf as libc::c_uint) << 16 as libc::c_uint) |
                (*TIM_BDTRInitStruct).BreakFilter
    }
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the BREAK2 input related BDTR bit-fields */
        tmpbdtr =
            tmpbdtr & !((0xf as libc::c_uint) << 20 as libc::c_uint) |
                (*TIM_BDTRInitStruct).Break2Filter;
        tmpbdtr =
            tmpbdtr & !((0x1 as libc::c_uint) << 24 as libc::c_uint) |
                (*TIM_BDTRInitStruct).Break2State;
        tmpbdtr =
            tmpbdtr & !((0x1 as libc::c_uint) << 25 as libc::c_uint) |
                (*TIM_BDTRInitStruct).Break2Polarity
    }
    /* Set TIMx_BDTR */
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t, tmpbdtr);
    return SUCCESS;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_tim.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   TIM LL module driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @addtogroup TIM_LL
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @addtogroup TIM_LL_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* * @defgroup TIM_LL_Private_Functions TIM Private Functions
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup TIM_LL_Private_Functions TIM Private Functions
 *  @brief   Private functions
  * @{
  */
/* *
  * @brief  Configure the TIMx output channel 1.
  * @param  TIMx Timer Instance
  * @param  TIM_OCInitStruct pointer to the the TIMx output channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn OC1Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_OCInitStruct:
                                   *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr1: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    let mut tmpcr2: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the Channel 1: Reset the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset Capture/Compare selection Bits */
    tmpccmr1 &= !((0x3 as libc::c_uint) << 0 as libc::c_uint);
    /* Set the Output Compare Mode */
    tmpccmr1 =
        tmpccmr1 & !((0x1007 as libc::c_uint) << 4 as libc::c_uint) |
            (*TIM_OCInitStruct).OCMode;
    /* Set the Output Compare Polarity */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 1 as libc::c_uint) |
            (*TIM_OCInitStruct).OCPolarity;
    /* Set the Output State */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 0 as libc::c_uint) |
            (*TIM_OCInitStruct).OCState;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the complementary output Polarity */
        tmpccer =
            tmpccer & !((0x1 as libc::c_uint) << 3 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNPolarity << 2 as libc::c_uint;
        /* Set the complementary output State */
        tmpccer =
            tmpccer & !((0x1 as libc::c_uint) << 2 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNState << 2 as libc::c_uint;
        /* Set the Output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 8 as libc::c_uint) |
                (*TIM_OCInitStruct).OCIdleState;
        /* Set the complementary output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 9 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNIdleState << 1 as libc::c_uint
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH1(TIMx, (*TIM_OCInitStruct).CompareValue);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx output channel 2.
  * @param  TIMx Timer Instance
  * @param  TIM_OCInitStruct pointer to the the TIMx output channel 2 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn OC2Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_OCInitStruct:
                                   *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr1: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    let mut tmpcr2: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset Capture/Compare selection Bits */
    tmpccmr1 &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmr1 =
        tmpccmr1 & !((0x1007 as libc::c_uint) << 12 as libc::c_uint) |
            (*TIM_OCInitStruct).OCMode << 8 as libc::c_uint;
    /* Set the Output Compare Polarity */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 5 as libc::c_uint) |
            (*TIM_OCInitStruct).OCPolarity << 4 as libc::c_uint;
    /* Set the Output State */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 4 as libc::c_uint) |
            (*TIM_OCInitStruct).OCState << 4 as libc::c_uint;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the complementary output Polarity */
        tmpccer =
            tmpccer & !((0x1 as libc::c_uint) << 7 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNPolarity << 6 as libc::c_uint;
        /* Set the complementary output State */
        tmpccer =
            tmpccer & !((0x1 as libc::c_uint) << 6 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNState << 6 as libc::c_uint;
        /* Set the Output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 10 as libc::c_uint) |
                (*TIM_OCInitStruct).OCIdleState << 2 as libc::c_uint;
        /* Set the complementary output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 11 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNIdleState << 3 as libc::c_uint
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH2(TIMx, (*TIM_OCInitStruct).CompareValue);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx output channel 3.
  * @param  TIMx Timer Instance
  * @param  TIM_OCInitStruct pointer to the the TIMx output channel 3 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn OC3Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_OCInitStruct:
                                   *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr2: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    let mut tmpcr2: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the Channel 3: Reset the CC3E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR2 register value */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset Capture/Compare selection Bits */
    tmpccmr2 &= !((0x3 as libc::c_uint) << 0 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmr2 =
        tmpccmr2 & !((0x1007 as libc::c_uint) << 4 as libc::c_uint) |
            (*TIM_OCInitStruct).OCMode;
    /* Set the Output Compare Polarity */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 9 as libc::c_uint) |
            (*TIM_OCInitStruct).OCPolarity << 8 as libc::c_uint;
    /* Set the Output State */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 8 as libc::c_uint) |
            (*TIM_OCInitStruct).OCState << 8 as libc::c_uint;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the complementary output Polarity */
        tmpccer =
            tmpccer & !((0x1 as libc::c_uint) << 11 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNPolarity << 10 as libc::c_uint;
        /* Set the complementary output State */
        tmpccer =
            tmpccer & !((0x1 as libc::c_uint) << 10 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNState << 10 as libc::c_uint;
        /* Set the Output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 12 as libc::c_uint) |
                (*TIM_OCInitStruct).OCIdleState << 4 as libc::c_uint;
        /* Set the complementary output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 13 as libc::c_uint) |
                (*TIM_OCInitStruct).OCNIdleState << 5 as libc::c_uint
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH3(TIMx, (*TIM_OCInitStruct).CompareValue);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx output channel 4.
  * @param  TIMx Timer Instance
  * @param  TIM_OCInitStruct pointer to the the TIMx output channel 4 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn OC4Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_OCInitStruct:
                                   *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr2: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    let mut tmpcr2: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the Channel 4: Reset the CC4E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           12 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR2 register value */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset Capture/Compare selection Bits */
    tmpccmr2 &= !((0x3 as libc::c_uint) << 8 as libc::c_uint);
    /* Select the Output Compare Mode */
    tmpccmr2 =
        tmpccmr2 & !((0x1007 as libc::c_uint) << 12 as libc::c_uint) |
            (*TIM_OCInitStruct).OCMode << 8 as libc::c_uint;
    /* Set the Output Compare Polarity */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 13 as libc::c_uint) |
            (*TIM_OCInitStruct).OCPolarity << 12 as libc::c_uint;
    /* Set the Output State */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 12 as libc::c_uint) |
            (*TIM_OCInitStruct).OCState << 12 as libc::c_uint;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the Output Idle state */
        tmpcr2 =
            tmpcr2 & !((0x1 as libc::c_uint) << 14 as libc::c_uint) |
                (*TIM_OCInitStruct).OCIdleState << 6 as libc::c_uint
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH4(TIMx, (*TIM_OCInitStruct).CompareValue);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx output channel 5.
  * @param  TIMx Timer Instance
  * @param  TIM_OCInitStruct pointer to the the TIMx output channel 5 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn OC5Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_OCInitStruct:
                                   *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr3: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the Channel 5: Reset the CC5E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           16 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CCMR3 register value */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Select the Output Compare Mode */
    tmpccmr3 =
        tmpccmr3 & !((0x1007 as libc::c_uint) << 4 as libc::c_uint) |
            (*TIM_OCInitStruct).OCMode;
    /* Set the Output Compare Polarity */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 17 as libc::c_uint) |
            (*TIM_OCInitStruct).OCPolarity << 16 as libc::c_uint;
    /* Set the Output State */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 16 as libc::c_uint) |
            (*TIM_OCInitStruct).OCState << 16 as libc::c_uint;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the Output Idle state */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (*TIMx).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              16 as libc::c_uint) |
                                        (*TIM_OCInitStruct).OCIdleState <<
                                            8 as libc::c_uint)
    }
    /* Write to TIMx CCMR3 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH5(TIMx, (*TIM_OCInitStruct).CompareValue);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx output channel 6.
  * @param  TIMx Timer Instance
  * @param  TIM_OCInitStruct pointer to the the TIMx output channel 6 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn OC6Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_OCInitStruct:
                                   *mut LL_TIM_OC_InitTypeDef)
 -> ErrorStatus {
    let mut tmpccmr3: uint32_t = 0 as libc::c_uint;
    let mut tmpccer: uint32_t = 0 as libc::c_uint;
    /* Check the parameters */
    /* Disable the Channel 5: Reset the CC6E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           20 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CCMR3 register value */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Select the Output Compare Mode */
    tmpccmr3 =
        tmpccmr3 & !((0x1007 as libc::c_uint) << 12 as libc::c_uint) |
            (*TIM_OCInitStruct).OCMode << 8 as libc::c_uint;
    /* Set the Output Compare Polarity */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 21 as libc::c_uint) |
            (*TIM_OCInitStruct).OCPolarity << 20 as libc::c_uint;
    /* Set the Output State */
    tmpccer =
        tmpccer & !((0x1 as libc::c_uint) << 20 as libc::c_uint) |
            (*TIM_OCInitStruct).OCState << 20 as libc::c_uint;
    if TIMx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0 as
                                                                              libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000 as
                    libc::c_uint).wrapping_add(0x10000 as
                                                   libc::c_uint).wrapping_add(0x400
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the Output Idle state */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (*TIMx).CR2 &
                                        !((0x1 as libc::c_uint) <<
                                              18 as libc::c_uint) |
                                        (*TIM_OCInitStruct).OCIdleState <<
                                            10 as libc::c_uint)
    }
    /* Write to TIMx CCMR3 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
    /* Set the Capture Compare Register value */
    LL_TIM_OC_SetCompareCH6(TIMx, (*TIM_OCInitStruct).CompareValue);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx input channel 1.
  * @param  TIMx Timer Instance
  * @param  TIM_ICInitStruct pointer to the the TIMx input channel 1 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn IC1Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_ICInitStruct:
                                   *mut LL_TIM_IC_InitTypeDef)
 -> ErrorStatus {
    /* Check the parameters */
    /* Disable the Channel 1: Reset the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           0 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Select the Input and set the filter and the prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                (*TIMx).CCMR1 &
                                    !((0x3 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0xf as libc::c_uint) <<
                                              4 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              2 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICActiveInput |
                                         (*TIM_ICInitStruct).ICFilter |
                                         (*TIM_ICInitStruct).ICPrescaler) >>
                                        16 as libc::c_uint);
    /* Select the Polarity and set the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (*TIMx).CCER &
                                    !((0x1 as libc::c_uint) <<
                                          1 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              3 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICPolarity |
                                         (0x1 as libc::c_uint) <<
                                             0 as libc::c_uint));
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx input channel 2.
  * @param  TIMx Timer Instance
  * @param  TIM_ICInitStruct pointer to the the TIMx input channel 2 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn IC2Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_ICInitStruct:
                                   *mut LL_TIM_IC_InitTypeDef)
 -> ErrorStatus {
    /* Check the parameters */
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           4 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Select the Input and set the filter and the prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                (*TIMx).CCMR1 &
                                    !((0x3 as libc::c_uint) <<
                                          8 as libc::c_uint |
                                          (0xf as libc::c_uint) <<
                                              12 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              10 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICActiveInput |
                                         (*TIM_ICInitStruct).ICFilter |
                                         (*TIM_ICInitStruct).ICPrescaler) >>
                                        8 as libc::c_uint);
    /* Select the Polarity and set the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (*TIMx).CCER &
                                    !((0x1 as libc::c_uint) <<
                                          5 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              7 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICPolarity <<
                                         4 as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             4 as libc::c_uint));
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx input channel 3.
  * @param  TIMx Timer Instance
  * @param  TIM_ICInitStruct pointer to the the TIMx input channel 3 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn IC3Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_ICInitStruct:
                                   *mut LL_TIM_IC_InitTypeDef)
 -> ErrorStatus {
    /* Check the parameters */
    /* Disable the Channel 3: Reset the CC3E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           8 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Select the Input and set the filter and the prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                (*TIMx).CCMR2 &
                                    !((0x3 as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0xf as libc::c_uint) <<
                                              4 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              2 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICActiveInput |
                                         (*TIM_ICInitStruct).ICFilter |
                                         (*TIM_ICInitStruct).ICPrescaler) >>
                                        16 as libc::c_uint);
    /* Select the Polarity and set the CC3E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (*TIMx).CCER &
                                    !((0x1 as libc::c_uint) <<
                                          9 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              11 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICPolarity <<
                                         8 as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             8 as libc::c_uint));
    return SUCCESS;
}
/* *
  * @brief  Configure the TIMx input channel 4.
  * @param  TIMx Timer Instance
  * @param  TIM_ICInitStruct pointer to the the TIMx input channel 4 configuration data structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: TIMx registers are de-initialized
  *          - ERROR: not applicable
  */
unsafe extern "C" fn IC4Config(mut TIMx: *mut TIM_TypeDef,
                               mut TIM_ICInitStruct:
                                   *mut LL_TIM_IC_InitTypeDef)
 -> ErrorStatus {
    /* Check the parameters */
    /* Disable the Channel 4: Reset the CC4E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           12 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    /* Select the Input and set the filter and the prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                (*TIMx).CCMR2 &
                                    !((0x3 as libc::c_uint) <<
                                          8 as libc::c_uint |
                                          (0xf as libc::c_uint) <<
                                              12 as libc::c_uint |
                                          (0x3 as libc::c_uint) <<
                                              10 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICActiveInput |
                                         (*TIM_ICInitStruct).ICFilter |
                                         (*TIM_ICInitStruct).ICPrescaler) >>
                                        8 as libc::c_uint);
    /* Select the Polarity and set the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (*TIMx).CCER &
                                    !((0x1 as libc::c_uint) <<
                                          13 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              15 as libc::c_uint) |
                                    ((*TIM_ICInitStruct).ICPolarity <<
                                         12 as libc::c_uint |
                                         (0x1 as libc::c_uint) <<
                                             12 as libc::c_uint));
    return SUCCESS;
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* TIM1 || TIM8 || TIM2 || TIM3 ||  TIM4 || TIM5 ||TIM9 || TIM10 || TIM11 || TIM12 || TIM13 || TIM14 || TIM6 || TIM7 */
/* *
  * @}
  */
/* *
  * @}
  */
