use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
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
  ******************************************************************************
  * @file    stm32f7xx_ll_tim.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of TIM LL module.
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
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @defgroup TIM_LL TIM
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @defgroup TIM_LL_Private_Variables TIM Private Variables
  * @{
  */
/* 0: TIMx_CH1  */
/* 1: TIMx_CH1N */
/* 2: TIMx_CH2  */
/* 3: TIMx_CH2N */
/* 4: TIMx_CH3  */
/* 5: TIMx_CH3N */
/* 6: TIMx_CH4  */
/* 7: TIMx_CH5  */
/* 8: TIMx_CH6  */
/* 0: OC1M, OC1FE, OC1PE */
/* 1: - NA */
/* 2: OC2M, OC2FE, OC2PE */
/* 3: - NA */
/* 4: OC3M, OC3FE, OC3PE */
/* 5: - NA */
/* 6: OC4M, OC4FE, OC4PE */
/* 7: OC5M, OC5FE, OC5PE */
/* 8: OC6M, OC6FE, OC6PE */
/* 0: CC1S, IC1PSC, IC1F */
/* 1: - NA */
/* 2: CC2S, IC2PSC, IC2F */
/* 3: - NA */
/* 4: CC3S, IC3PSC, IC3F */
/* 5: - NA */
/* 6: CC4S, IC4PSC, IC4F */
/* 7: - NA */
/* 8: - NA */
/* 0: CC1P */
/* 1: CC1NP */
/* 2: CC2P */
/* 3: CC2NP */
/* 4: CC3P */
/* 5: CC3NP */
/* 6: CC4P */
/* 7: CC5P */
/* 8: CC6P */
/* 0: OIS1 */
/* 1: OIS1N */
/* 2: OIS2 */
/* 3: OIS2N */
/* 4: OIS3 */
/* 5: OIS3N */
/* 6: OIS4 */
/* 7: OIS5 */
/* 8: OIS6 */
/* *
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/* * @defgroup TIM_LL_Private_Constants TIM Private Constants
  * @{
  */
/* TIM_BREAK_INPUT_SUPPORT */
/* Remap mask definitions */
/* Mask used to set the TDG[x:0] of the DTG bits of the TIMx_BDTR register */
/* Mask used to set the DTG[7:5] bits of the DTG bits of the TIMx_BDTR register */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup TIM_LL_Private_Macros TIM Private Macros
  * @{
  */
/* * @brief  Convert channel id into channel index.
  * @param  __CHANNEL__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval none
  */
/* * @brief  Calculate the deadtime sampling period(in ps).
  * @param  __TIMCLK__ timer input clock frequency (in Hz).
  * @param  __CKD__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @retval none
  */
/* *
  * @}
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup TIM_LL_ES_INIT TIM Exported Init structure
  * @{
  */
/* *
  * @brief  TIM Time Base configuration structure definition.
  */
/* !< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                                   This feature can be modified afterwards using unitary function @ref LL_TIM_SetPrescaler().*/
/* !< Specifies the counter mode.
                                   This parameter can be a value of @ref TIM_LL_EC_COUNTERMODE.

                                   This feature can be modified afterwards using unitary function @ref LL_TIM_SetCounterMode().*/
/* !< Specifies the auto reload value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter must be a number between Min_Data=0x0000 and Max_Data=0xFFFF.
                                   Some timer instances may support 32 bits counters. In that case this parameter must be a number between 0x0000 and 0xFFFFFFFF.

                                   This feature can be modified afterwards using unitary function @ref LL_TIM_SetAutoReload().*/
/* !< Specifies the clock division.
                                   This parameter can be a value of @ref TIM_LL_EC_CLOCKDIVISION.

                                   This feature can be modified afterwards using unitary function @ref LL_TIM_SetClockDivision().*/
/* !< Specifies the repetition counter value. Each time the RCR downcounter
                                   reaches zero, an update event is generated and counting restarts
                                   from the RCR value (N).
                                   This means in PWM mode that (N+1) corresponds to:
                                      - the number of PWM periods in edge-aligned mode
                                      - the number of half PWM period in center-aligned mode
                                   This parameter must be a number between 0x00 and 0xFF.

                                   This feature can be modified afterwards using unitary function @ref LL_TIM_SetRepetitionCounter().*/
/* *
  * @brief  TIM Output Compare configuration structure definition.
  */
/* !< Specifies the output mode.
                               This parameter can be a value of @ref TIM_LL_EC_OCMODE.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetMode().*/
/* !< Specifies the TIM Output Compare state.
                               This parameter can be a value of @ref TIM_LL_EC_OCSTATE.

                               This feature can be modified afterwards using unitary functions @ref LL_TIM_CC_EnableChannel() or @ref LL_TIM_CC_DisableChannel().*/
/* !< Specifies the TIM complementary Output Compare state.
                               This parameter can be a value of @ref TIM_LL_EC_OCSTATE.

                               This feature can be modified afterwards using unitary functions @ref LL_TIM_CC_EnableChannel() or @ref LL_TIM_CC_DisableChannel().*/
/* !< Specifies the Compare value to be loaded into the Capture Compare Register.
                               This parameter can be a number between Min_Data=0x0000 and Max_Data=0xFFFF.

                               This feature can be modified afterwards using unitary function LL_TIM_OC_SetCompareCHx (x=1..6).*/
/* !< Specifies the output polarity.
                               This parameter can be a value of @ref TIM_LL_EC_OCPOLARITY.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetPolarity().*/
/* !< Specifies the complementary output polarity.
                               This parameter can be a value of @ref TIM_LL_EC_OCPOLARITY.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetPolarity().*/
/* !< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_LL_EC_OCIDLESTATE.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetIdleState().*/
/* !< Specifies the TIM Output Compare pin state during Idle state.
                               This parameter can be a value of @ref TIM_LL_EC_OCIDLESTATE.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetIdleState().*/
/* *
  * @brief  TIM Input Capture configuration structure definition.
  */
/* !< Specifies the active edge of the input signal.
                               This parameter can be a value of @ref TIM_LL_EC_IC_POLARITY.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPolarity().*/
/* !< Specifies the input.
                               This parameter can be a value of @ref TIM_LL_EC_ACTIVEINPUT.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetActiveInput().*/
/* !< Specifies the Input Capture Prescaler.
                               This parameter can be a value of @ref TIM_LL_EC_ICPSC.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPrescaler().*/
/* !< Specifies the input capture filter.
                               This parameter can be a value of @ref TIM_LL_EC_IC_FILTER.

                               This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetFilter().*/
/* *
  * @brief  TIM Encoder interface configuration structure definition.
  */
/* !< Specifies the encoder resolution (x2 or x4).
                                 This parameter can be a value of @ref TIM_LL_EC_ENCODERMODE.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_SetEncoderMode().*/
/* !< Specifies the active edge of TI1 input.
                                 This parameter can be a value of @ref TIM_LL_EC_IC_POLARITY.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPolarity().*/
/* !< Specifies the TI1 input source
                                 This parameter can be a value of @ref TIM_LL_EC_ACTIVEINPUT.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetActiveInput().*/
/* !< Specifies the TI1 input prescaler value.
                                 This parameter can be a value of @ref TIM_LL_EC_ICPSC.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPrescaler().*/
/* !< Specifies the TI1 input filter.
                                 This parameter can be a value of @ref TIM_LL_EC_IC_FILTER.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetFilter().*/
/* !< Specifies the active edge of TI2 input.
                                 This parameter can be a value of @ref TIM_LL_EC_IC_POLARITY.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPolarity().*/
/* !< Specifies the TI2 input source
                                 This parameter can be a value of @ref TIM_LL_EC_ACTIVEINPUT.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetActiveInput().*/
/* !< Specifies the TI2 input prescaler value.
                                 This parameter can be a value of @ref TIM_LL_EC_ICPSC.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPrescaler().*/
/* !< Specifies the TI2 input filter.
                                 This parameter can be a value of @ref TIM_LL_EC_IC_FILTER.

                                 This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetFilter().*/
/* *
  * @brief  TIM Hall sensor interface configuration structure definition.
  */
/* !< Specifies the active edge of TI1 input.
                                    This parameter can be a value of @ref TIM_LL_EC_IC_POLARITY.

                                    This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPolarity().*/
/* !< Specifies the TI1 input prescaler value.
                                    Prescaler must be set to get a maximum counter period longer than the
                                    time interval between 2 consecutive changes on the Hall inputs.
                                    This parameter can be a value of @ref TIM_LL_EC_ICPSC.

                                    This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetPrescaler().*/
/* !< Specifies the TI1 input filter.
                                    This parameter can be a value of @ref TIM_LL_EC_IC_FILTER.

                                    This feature can be modified afterwards using unitary function @ref LL_TIM_IC_SetFilter().*/
/* !< Specifies the compare value to be loaded into the Capture Compare Register.
                                    A positive pulse (TRGO event) is generated with a programmable delay every time
                                    a change occurs on the Hall inputs.
                                    This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.

                                    This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetCompareCH2().*/
/* * 
  * @brief  BDTR (Break and Dead Time) structure definition 
  */
/* !< Specifies the Off-State selection used in Run mode.
                                      This parameter can be a value of @ref TIM_LL_EC_OSSR

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_SetOffStates()
  
                                      @note This bit-field cannot be modified as long as LOCK level 2 has been programmed. */
/* !< Specifies the Off-State used in Idle state.
                                      This parameter can be a value of @ref TIM_LL_EC_OSSI

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_SetOffStates()
  
                                      @note This bit-field cannot be modified as long as LOCK level 2 has been programmed. */
/* !< Specifies the LOCK level parameters.
                                      This parameter can be a value of @ref TIM_LL_EC_LOCKLEVEL
  
                                      @note The LOCK bits can be written only once after the reset. Once the TIMx_BDTR register
                                            has been written, their content is frozen until the next reset.*/
/* !< Specifies the delay time between the switching-off and the
                                      switching-on of the outputs.
                                      This parameter can be a number between Min_Data = 0x00 and Max_Data = 0xFF.

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_OC_SetDeadTime()
  
                                      @note This bit-field can not be modified as long as LOCK level 1, 2 or 3 has been programmed. */
/* !< Specifies whether the TIM Break input is enabled or not. 
                                      This parameter can be a value of @ref TIM_LL_EC_BREAK_ENABLE

                                      This feature can be modified afterwards using unitary functions @ref LL_TIM_EnableBRK() or @ref LL_TIM_DisableBRK()

                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* !< Specifies the TIM Break Input pin polarity.
                                      This parameter can be a value of @ref TIM_LL_EC_BREAK_POLARITY

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_ConfigBRK()
  
                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* !< Specifies the TIM Break Filter.
                                      This parameter can be a value of @ref TIM_LL_EC_BREAK_FILTER

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_ConfigBRK()
  
                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* !< Specifies whether the TIM Break2 input is enabled or not. 
                                      This parameter can be a value of @ref TIM_LL_EC_BREAK2_ENABLE

                                      This feature can be modified afterwards using unitary functions @ref LL_TIM_EnableBRK2() or @ref LL_TIM_DisableBRK2()

                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* !< Specifies the TIM Break2 Input pin polarity.
                                      This parameter can be a value of @ref TIM_LL_EC_BREAK2_POLARITY

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_ConfigBRK2()
  
                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* !< Specifies the TIM Break2 Filter.
                                      This parameter can be a value of @ref TIM_LL_EC_BREAK2_FILTER

                                      This feature can be modified afterwards using unitary function @ref LL_TIM_ConfigBRK2()
  
                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* !< Specifies whether the TIM Automatic Output feature is enabled or not. 
                                      This parameter can be a value of @ref TIM_LL_EC_AUTOMATICOUTPUT_ENABLE

                                      This feature can be modified afterwards using unitary functions @ref LL_TIM_EnableAutomaticOutput() or @ref LL_TIM_DisableAutomaticOutput()
  
                                      @note This bit-field can not be modified as long as LOCK level 1 has been programmed. */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup TIM_LL_Exported_Constants TIM Exported Constants
  * @{
  */
/* * @defgroup TIM_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_TIM_ReadReg function.
  * @{
  */
/* !< Update interrupt flag */
/* !< Capture/compare 1 interrupt flag */
/* !< Capture/compare 2 interrupt flag */
/* !< Capture/compare 3 interrupt flag */
/* !< Capture/compare 4 interrupt flag */
/* !< Capture/compare 5 interrupt flag */
/* !< Capture/compare 6 interrupt flag */
/* !< COM interrupt flag */
/* !< Trigger interrupt flag */
/* !< Break interrupt flag */
/* !< Second break interrupt flag */
/* !< Capture/Compare 1 overcapture flag */
/* !< Capture/Compare 2 overcapture flag */
/* !< Capture/Compare 3 overcapture flag */
/* !< Capture/Compare 4 overcapture flag */
/* !< System Break interrupt flag  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_BREAK_ENABLE Break Enable
  * @{
  */
/* !< Break function disabled */
/* !< Break function enabled */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_BREAK2_ENABLE Break2 Enable
  * @{
  */
/* !< Break2 function disabled */
/* !< Break2 function enabled */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_AUTOMATICOUTPUT_ENABLE Automatic output enable
  * @{
  */
/* !< MOE can be set only by software */
/* !< MOE can be set by software or automatically at the next update event */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* * @defgroup TIM_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_TIM_ReadReg and  LL_TIM_WriteReg functions.
  * @{
  */
/* !< Update interrupt enable */
/* !< Capture/compare 1 interrupt enable */
/* !< Capture/compare 2 interrupt enable */
/* !< Capture/compare 3 interrupt enable */
/* !< Capture/compare 4 interrupt enable */
/* !< COM interrupt enable */
/* !< Trigger interrupt enable */
/* !< Break interrupt enable */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_UPDATESOURCE Update Source
  * @{
  */
/* !< Counter overflow/underflow, Setting the UG bit or Update generation through the slave mode controller generates an update request */
/* !< Only counter overflow/underflow generates an update request */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ONEPULSEMODE One Pulse Mode
  * @{
  */
/* !< Counter is not stopped at update event */
/* !< Counter stops counting at the next update event */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_COUNTERMODE Counter Mode
  * @{
  */
/* !<Counter used as upcounter */
/* !< Counter used as downcounter */
/* !< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting down. */
/* !<The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up */
/* !< The counter counts up and down alternatively. Output compare interrupt flags of output channels  are set only when the counter is counting up or down. */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_CLOCKDIVISION Clock Division
  * @{
  */
/* !< tDTS=tCK_INT */
/* !< tDTS=2*tCK_INT */
/* !< tDTS=4*tCK_INT */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_COUNTERDIRECTION Counter Direction
  * @{
  */
/* !< Timer counter counts up */
/* !< Timer counter counts down */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_CCUPDATESOURCE Capture Compare  Update Source
  * @{
  */
/* !< Capture/compare control bits are updated by setting the COMG bit only */
/* !< Capture/compare control bits are updated by setting the COMG bit or when a rising edge occurs on trigger input (TRGI) */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_CCDMAREQUEST Capture Compare DMA Request
  * @{
  */
/* !< CCx DMA request sent when CCx event occurs */
/* !< CCx DMA requests sent when update event occurs */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_LOCKLEVEL Lock Level
  * @{
  */
/* !< LOCK OFF - No bit is write protected */
/* !< LOCK Level 1 */
/* !< LOCK Level 2 */
/* !< LOCK Level 3 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_CHANNEL Channel
  * @{
  */
/* !< Timer input/output channel 1 */
/* !< Timer complementary output channel 1 */
/* !< Timer input/output channel 2 */
/* !< Timer complementary output channel 2 */
/* !< Timer input/output channel 3 */
/* !< Timer complementary output channel 3 */
/* !< Timer input/output channel 4 */
/* !< Timer output channel 5 */
/* !< Timer output channel 6 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_OCSTATE Output Configuration State
  * @{
  */
/* !< OCx is not active */
/* !< OCx signal is output on the corresponding output pin */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* * @defgroup TIM_LL_EC_OCMODE Output Configuration Mode
  * @{
  */
/* !<The comparison between the output compare register TIMx_CCRy and the counter TIMx_CNT has no effect on the output channel level */
/* !<OCyREF is forced high on compare match*/
/* !<OCyREF is forced low on compare match*/
/* !<OCyREF toggles on compare match*/
/* !<OCyREF is forced low*/
/* !<OCyREF is forced high*/
/* !<In upcounting, channel y is active as long as TIMx_CNT<TIMx_CCRy else inactive.  In downcounting, channel y is inactive as long as TIMx_CNT>TIMx_CCRy else active.*/
/* !<In upcounting, channel y is inactive as long as TIMx_CNT<TIMx_CCRy else active.  In downcounting, channel y is active as long as TIMx_CNT>TIMx_CCRy else inactive*/
/* !<Retrigerrable OPM mode 1*/
/* !<Retrigerrable OPM mode 2*/
/* !<Combined PWM mode 1*/
/* !<Combined PWM mode 2*/
/* !<Asymmetric PWM mode 1*/
/* !<Asymmetric PWM mode 2*/
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_OCPOLARITY Output Configuration Polarity
  * @{
  */
/* !< OCxactive high*/
/* !< OCxactive low*/
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_OCIDLESTATE Output Configuration Idle State
  * @{
  */
/* !<OCx=0 (after a dead-time if OC is implemented) when MOE=0*/
/* !<OCx=1 (after a dead-time if OC is implemented) when MOE=0*/
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_GROUPCH5 GROUPCH5
  * @{
  */
/* !< No effect of OC5REF on OC1REFC, OC2REFC and OC3REFC */
/* !< OC1REFC is the logical AND of OC1REFC and OC5REF */
/* !< OC2REFC is the logical AND of OC2REFC and OC5REF */
/* !< OC3REFC is the logical AND of OC3REFC and OC5REF */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ACTIVEINPUT Active Input Selection
  * @{
  */
/* !< ICx is mapped on TIx */
/* !< ICx is mapped on TIy */
/* !< ICx is mapped on TRC */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ICPSC Input Configuration Prescaler
  * @{
  */
/* !< No prescaler, capture is done each time an edge is detected on the capture input */
/* !< Capture is done once every 2 events */
/* !< Capture is done once every 4 events */
/* !< Capture is done once every 8 events */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_IC_FILTER Input Configuration Filter
  * @{
  */
/* !< No filter, sampling is done at fDTS */
/* !< fSAMPLING=fCK_INT, N=2 */
/* !< fSAMPLING=fCK_INT, N=4 */
/* !< fSAMPLING=fCK_INT, N=8 */
/* !< fSAMPLING=fDTS/2, N=6 */
/* !< fSAMPLING=fDTS/2, N=8 */
/* !< fSAMPLING=fDTS/4, N=6 */
/* !< fSAMPLING=fDTS/4, N=8 */
/* !< fSAMPLING=fDTS/8, N=6 */
/* !< fSAMPLING=fDTS/8, N=8 */
/* !< fSAMPLING=fDTS/16, N=5 */
/* !< fSAMPLING=fDTS/16, N=6 */
/* !< fSAMPLING=fDTS/16, N=8 */
/* !< fSAMPLING=fDTS/32, N=5 */
/* !< fSAMPLING=fDTS/32, N=6 */
/* !< fSAMPLING=fDTS/32, N=8 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_IC_POLARITY Input Configuration Polarity
  * @{
  */
/* !< The circuit is sensitive to TIxFP1 rising edge, TIxFP1 is not inverted */
/* !< The circuit is sensitive to TIxFP1 falling edge, TIxFP1 is inverted */
/* !< The circuit is sensitive to both TIxFP1 rising and falling edges, TIxFP1 is not inverted */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_CLOCKSOURCE Clock Source
  * @{
  */
/* !< The timer is clocked by the internal clock provided from the RCC */
/* !< Counter counts at each rising or falling edge on a selected inpu t*/
/* !< Counter counts at each rising or falling edge on the external trigger input ETR */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ENCODERMODE Encoder Mode
  * @{
  */
/* !< Encoder mode 1 - Counter counts up/down on TI2FP2 edge depending on TI1FP1 level */
/* !< Encoder mode 2 - Counter counts up/down on TI1FP1 edge depending on TI2FP2 level */
/* !< Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges                                                                                                                                                                   depending on the level of the other input l */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TRGO Trigger Output
  * @{
  */
/* !< UG bit from the TIMx_EGR register is used as trigger output */
/* !< Counter Enable signal (CNT_EN) is used as trigger output */
/* !< Update event is used as trigger output */
/* !< CC1 capture or a compare match is used as trigger output */
/* !< OC1REF signal is used as trigger output */
/* !< OC2REF signal is used as trigger output */
/* !< OC3REF signal is used as trigger output */
/* !< OC4REF signal is used as trigger output */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TRGO2 Trigger Output 2
  * @{
  */
/* !< UG bit from the TIMx_EGR register is used as trigger output 2 */
/* !< Counter Enable signal (CNT_EN) is used as trigger output 2 */
/* !< Update event is used as trigger output 2 */
/* !< CC1 capture or a compare match is used as trigger output 2 */
/* !< OC1REF signal is used as trigger output 2 */
/* !< OC2REF signal is used as trigger output 2 */
/* !< OC3REF signal is used as trigger output 2 */
/* !< OC4REF signal is used as trigger output 2 */
/* !< OC5REF signal is used as trigger output 2 */
/* !< OC6REF signal is used as trigger output 2 */
/* !< OC4REF rising or falling edges are used as trigger output 2 */
/* !< OC6REF rising or falling edges are used as trigger output 2 */
/* !< OC4REF or OC6REF rising edges are used as trigger output 2 */
/* !< OC4REF rising or OC6REF falling edges are used as trigger output 2 */
/* !< OC5REF or OC6REF rising edges are used as trigger output 2 */
/* !< OC5REF rising or OC6REF falling edges are used as trigger output 2 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_SLAVEMODE Slave Mode
  * @{
  */
/* !< Slave mode disabled */
/* !< Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter */
/* !< Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high */
/* !< Trigger Mode - The counter starts at a rising edge of the trigger TRGI */
/* !< Combined reset + trigger mode - Rising edge of the selected trigger input (TRGI)  reinitializes the counter, generates an update of the registers and starts the counter */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TS Trigger Selection
  * @{
  */
/* !< Internal Trigger 0 (ITR0) is used as trigger input */
/* !< Internal Trigger 1 (ITR1) is used as trigger input */
/* !< Internal Trigger 2 (ITR2) is used as trigger input */
/* !< Internal Trigger 3 (ITR3) is used as trigger input */
/* !< TI1 Edge Detector (TI1F_ED) is used as trigger input */
/* !< Filtered Timer Input 1 (TI1FP1) is used as trigger input */
/* !< Filtered Timer Input 2 (TI12P2) is used as trigger input */
/* !< Filtered external Trigger (ETRF) is used as trigger input */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ETR_POLARITY External Trigger Polarity
  * @{
  */
/* !< ETR is non-inverted, active at high level or rising edge */
/* !< ETR is inverted, active at low level or falling edge */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ETR_PRESCALER External Trigger Prescaler
  * @{
  */
/* !< ETR prescaler OFF */
/* !< ETR frequency is divided by 2 */
/* !< ETR frequency is divided by 4 */
/* !< ETR frequency is divided by 8 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_ETR_FILTER External Trigger Filter
  * @{
  */
/* !< No filter, sampling is done at fDTS */
/* !< fSAMPLING=fCK_INT, N=2 */
/* !< fSAMPLING=fCK_INT, N=4 */
/* !< fSAMPLING=fCK_INT, N=8 */
/* !< fSAMPLING=fDTS/2, N=6 */
/* !< fSAMPLING=fDTS/2, N=8 */
/* !< fSAMPLING=fDTS/4, N=6 */
/* !< fSAMPLING=fDTS/4, N=8 */
/* !< fSAMPLING=fDTS/8, N=8 */
/* !< fSAMPLING=fDTS/16, N=5 */
/* !< fSAMPLING=fDTS/16, N=6 */
/* !< fSAMPLING=fDTS/16, N=8 */
/* !< fSAMPLING=fDTS/16, N=5 */
/* !< fSAMPLING=fDTS/32, N=5 */
/* !< fSAMPLING=fDTS/32, N=6 */
/* !< fSAMPLING=fDTS/32, N=8 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_BREAK_POLARITY break polarity
  * @{
  */
/* !< Break input BRK is active low */
/* !< Break input BRK is active high */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_BREAK_FILTER break filter
  * @{
  */
/* !< No filter, BRK acts asynchronously */
/* !< fSAMPLING=fCK_INT, N=2 */
/* !< fSAMPLING=fCK_INT, N=4 */
/* !< fSAMPLING=fCK_INT, N=8 */
/* !< fSAMPLING=fDTS/2, N=6 */
/* !< fSAMPLING=fDTS/2, N=8 */
/* !< fSAMPLING=fDTS/4, N=6 */
/* !< fSAMPLING=fDTS/4, N=8 */
/* !< fSAMPLING=fDTS/8, N=6 */
/* !< fSAMPLING=fDTS/8, N=8 */
/* !< fSAMPLING=fDTS/16, N=5 */
/* !< fSAMPLING=fDTS/16, N=6 */
/* !< fSAMPLING=fDTS/16, N=8 */
/* !< fSAMPLING=fDTS/32, N=5 */
/* !< fSAMPLING=fDTS/32, N=6 */
/* !< fSAMPLING=fDTS/32, N=8 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_BREAK2_POLARITY BREAK2 POLARITY
  * @{
  */
/* !< Break input BRK2 is active low */
/* !< Break input BRK2 is active high */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_BREAK2_FILTER BREAK2 FILTER
  * @{
  */
/* !< No filter, BRK acts asynchronously */
/* !< fSAMPLING=fCK_INT, N=2 */
/* !< fSAMPLING=fCK_INT, N=4 */
/* !< fSAMPLING=fCK_INT, N=8 */
/* !< fSAMPLING=fDTS/2, N=6 */
/* !< fSAMPLING=fDTS/2, N=8 */
/* !< fSAMPLING=fDTS/4, N=6 */
/* !< fSAMPLING=fDTS/4, N=8 */
/* !< fSAMPLING=fDTS/8, N=6 */
/* !< fSAMPLING=fDTS/8, N=8 */
/* !< fSAMPLING=fDTS/16, N=5 */
/* !< fSAMPLING=fDTS/16, N=6 */
/* !< fSAMPLING=fDTS/16, N=8 */
/* !< fSAMPLING=fDTS/32, N=5 */
/* !< fSAMPLING=fDTS/32, N=6 */
/* !< fSAMPLING=fDTS/32, N=8 */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_OSSI OSSI
  * @{
  */
/* !< When inactive, OCx/OCxN outputs are disabled */
/* !< When inactive, OxC/OCxN outputs are first forced with their inactive level then forced to their idle level after the deadtime */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_OSSR OSSR
  * @{
  */
/* !< When inactive, OCx/OCxN outputs are disabled */
/* !< When inactive, OC/OCN outputs are enabled with their inactive level as soon as CCxE=1 or CCxNE=1 */
/* *
  * @}
  */
/* TIM_BREAK_INPUT_SUPPORT */
/* * @defgroup TIM_LL_EC_DMABURST_BASEADDR DMA Burst Base Address
  * @{
  */
/* !< TIMx_CR1 register is the DMA base address for DMA burst */
/* !< TIMx_CR2 register is the DMA base address for DMA burst */
/* !< TIMx_SMCR register is the DMA base address for DMA burst */
/* !< TIMx_DIER register is the DMA base address for DMA burst */
/* !< TIMx_SR register is the DMA base address for DMA burst */
/* !< TIMx_EGR register is the DMA base address for DMA burst */
/* !< TIMx_CCMR1 register is the DMA base address for DMA burst */
/* !< TIMx_CCMR2 register is the DMA base address for DMA burst */
/* !< TIMx_CCER register is the DMA base address for DMA burst */
/* !< TIMx_CNT register is the DMA base address for DMA burst */
/* !< TIMx_PSC register is the DMA base address for DMA burst */
/* !< TIMx_ARR register is the DMA base address for DMA burst */
/* !< TIMx_RCR register is the DMA base address for DMA burst */
/* !< TIMx_CCR1 register is the DMA base address for DMA burst */
/* !< TIMx_CCR2 register is the DMA base address for DMA burst */
/* !< TIMx_CCR3 register is the DMA base address for DMA burst */
/* !< TIMx_CCR4 register is the DMA base address for DMA burst */
/* !< TIMx_BDTR register is the DMA base address for DMA burst */
/* !< TIMx_CCMR3 register is the DMA base address for DMA burst */
/* !< TIMx_CCR5 register is the DMA base address for DMA burst */
/* !< TIMx_CCR6 register is the DMA base address for DMA burst */
/* !< TIMx_OR register is the DMA base address for DMA burst */
/* !< TIMx_AF1 register is the DMA base address for DMA burst */
/* !< TIMx_AF2 register is the DMA base address for DMA burst */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_DMABURST_LENGTH DMA Burst Length
  * @{
  */
/* !< Transfer is done to 1 register starting from the DMA burst base address */
/* !< Transfer is done to 2 registers starting from the DMA burst base address */
/* !< Transfer is done to 3 registers starting from the DMA burst base address */
/* !< Transfer is done to 4 registers starting from the DMA burst base address */
/* !< Transfer is done to 5 registers starting from the DMA burst base address */
/* !< Transfer is done to 6 registers starting from the DMA burst base address */
/* !< Transfer is done to 7 registers starting from the DMA burst base address */
/* !< Transfer is done to 1 registers starting from the DMA burst base address */
/* !< Transfer is done to 9 registers starting from the DMA burst base address */
/* !< Transfer is done to 10 registers starting from the DMA burst base address */
/* !< Transfer is done to 11 registers starting from the DMA burst base address */
/* !< Transfer is done to 12 registers starting from the DMA burst base address */
/* !< Transfer is done to 13 registers starting from the DMA burst base address */
/* !< Transfer is done to 14 registers starting from the DMA burst base address */
/* !< Transfer is done to 15 registers starting from the DMA burst base address */
/* !< Transfer is done to 16 registers starting from the DMA burst base address */
/* !< Transfer is done to 17 registers starting from the DMA burst base address */
/* !< Transfer is done to 18 registers starting from the DMA burst base address */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TIM2_ITR1_RMP_TIM8  TIM2 Internal Trigger1 Remap TIM8
  * @{
  */
/* !< TIM2_ITR1 is connected to TIM8_TRGO */
/* !< TIM2_ITR1 is connected to ETH_PTP */
/* !< TIM2_ITR1 is connected to OTG_FS SOF */
/* !< TIM2_ITR1 is connected to OTG_HS SOF */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TIM5_TI4_RMP  TIM5 External Input Ch4 Remap
  * @{
  */
/* !< TIM5 channel 4 is connected to GPIO */
/* !< TIM5 channel 4 is connected to LSI internal clock */
/* !< TIM5 channel 4 is connected to LSE */
/* !< TIM5 channel 4 is connected to RTC wakeup interrupt */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EC_TIM11_TI1_RMP  TIM11 External Input Capture 1 Remap
  * @{
  */
/* !< TIM11 channel 1 is connected to GPIO */
/* !< TIM11 channel 1 is connected to SPDIFRX */
/* !< TIM11 channel 1 is connected to HSE */
/* !< TIM11 channel 1 is connected to MCO1 */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup TIM_LL_Exported_Macros TIM Exported Macros
  * @{
  */
/* * @defgroup TIM_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in TIM register.
  * @param  __INSTANCE__ TIM Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in TIM register.
  * @param  __INSTANCE__ TIM Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EM_Exported_Macros Exported_Macros
  * @{
  */
/* *
  * @brief  HELPER macro retrieving the UIFCPY flag from the counter value.
  * @note ex: @ref __LL_TIM_GETFLAG_UIFCPY (@ref LL_TIM_GetCounter ());
  * @note  Relevant only if UIF flag remapping has been enabled  (UIF status bit is copied
  *        to TIMx_CNT register bit 31)
  * @param  __CNT__ Counter value
  * @retval UIF status bit
  */
/* *
  * @brief  HELPER macro calculating DTG[0:7] in the TIMx_BDTR register to achieve the requested dead time duration.
  * @note ex: @ref __LL_TIM_CALC_DEADTIME (80000000, @ref LL_TIM_GetClockDivision (), 120);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __CKD__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @param  __DT__ deadtime duration (in ns)
  * @retval DTG[0:7]
  */
/* *
  * @brief  HELPER macro calculating the prescaler value to achieve the required counter clock frequency.
  * @note ex: @ref __LL_TIM_CALC_PSC (80000000, 1000000);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __CNTCLK__ counter clock frequency (in Hz)
  * @retval Prescaler value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro calculating the auto-reload value to achieve the required output signal frequency.
  * @note ex: @ref __LL_TIM_CALC_ARR (1000000, @ref LL_TIM_GetPrescaler (), 10000);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __FREQ__ output signal frequency (in Hz)
  * @retval  Auto-reload value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro calculating the compare value required to achieve the required timer output compare active/inactive delay.
  * @note ex: @ref __LL_TIM_CALC_DELAY (1000000, @ref LL_TIM_GetPrescaler (), 10);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @retval Compare value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro calculating the auto-reload value to achieve the required pulse duration (when the timer operates in one pulse mode).
  * @note ex: @ref __LL_TIM_CALC_PULSE (1000000, @ref LL_TIM_GetPrescaler (), 10, 20);
  * @param  __TIMCLK__ timer input clock frequency (in Hz)
  * @param  __PSC__ prescaler
  * @param  __DELAY__ timer output compare active/inactive delay (in us)
  * @param  __PULSE__ pulse duration (in us)
  * @retval Auto-reload value  (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  HELPER macro retrieving the ratio of the input capture prescaler
  * @note ex: @ref __LL_TIM_GET_ICPSC_RATIO (@ref LL_TIM_IC_GetPrescaler ());
  * @param  __ICPSC__ This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  * @retval Input capture prescaler ratio (1, 2, 4 or 8)
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup TIM_LL_Exported_Functions TIM Exported Functions
  * @{
  */
/* * @defgroup TIM_LL_EF_Time_Base Time Base configuration
  * @{
  */
/* *
  * @brief  Enable timer counter.
  * @rmtoll CR1          CEN           LL_TIM_EnableCounter
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable timer counter.
  * @rmtoll CR1          CEN           LL_TIM_DisableCounter
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the timer counter is enabled.
  * @rmtoll CR1          CEN           LL_TIM_IsEnabledCounter
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable update event generation.
  * @rmtoll CR1          UDIS          LL_TIM_EnableUpdateEvent
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable update event generation.
  * @rmtoll CR1          UDIS          LL_TIM_DisableUpdateEvent
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether update event generation is enabled.
  * @rmtoll CR1          UDIS          LL_TIM_IsEnabledUpdateEvent
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set update event source
  * @note Update event source set to LL_TIM_UPDATESOURCE_REGULAR: any of the following events
  *       generate an update interrupt or DMA request if enabled:
  *        - Counter overflow/underflow
  *        - Setting the UG bit
  *        - Update generation through the slave mode controller
  * @note Update event source set to LL_TIM_UPDATESOURCE_COUNTER: only counter
  *       overflow/underflow generates an update interrupt or DMA request if enabled.
  * @rmtoll CR1          URS           LL_TIM_SetUpdateSource
  * @param  TIMx Timer instance
  * @param  UpdateSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_UPDATESOURCE_REGULAR
  *         @arg @ref LL_TIM_UPDATESOURCE_COUNTER
  * @retval None
  */
/* *
  * @brief  Get actual event update source
  * @rmtoll CR1          URS           LL_TIM_GetUpdateSource
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_UPDATESOURCE_REGULAR
  *         @arg @ref LL_TIM_UPDATESOURCE_COUNTER
  */
/* *
  * @brief  Set one pulse mode (one shot v.s. repetitive).
  * @rmtoll CR1          OPM           LL_TIM_SetOnePulseMode
  * @param  TIMx Timer instance
  * @param  OnePulseMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ONEPULSEMODE_SINGLE
  *         @arg @ref LL_TIM_ONEPULSEMODE_REPETITIVE
  * @retval None
  */
/* *
  * @brief  Get actual one pulse mode.
  * @rmtoll CR1          OPM           LL_TIM_GetOnePulseMode
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ONEPULSEMODE_SINGLE
  *         @arg @ref LL_TIM_ONEPULSEMODE_REPETITIVE
  */
/* *
  * @brief  Set the timer counter counting mode.
  * @note Macro @ref IS_TIM_COUNTER_MODE_SELECT_INSTANCE(TIMx) can be used to
  *       check whether or not the counter mode selection feature is supported
  *       by a timer instance.
  * @rmtoll CR1          DIR           LL_TIM_SetCounterMode\n
  *         CR1          CMS           LL_TIM_SetCounterMode
  * @param  TIMx Timer instance
  * @param  CounterMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERMODE_UP
  *         @arg @ref LL_TIM_COUNTERMODE_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP_DOWN
  * @retval None
  */
/* *
  * @brief  Get actual counter mode.
  * @note Macro @ref IS_TIM_COUNTER_MODE_SELECT_INSTANCE(TIMx) can be used to
  *       check whether or not the counter mode selection feature is supported
  *       by a timer instance.
  * @rmtoll CR1          DIR           LL_TIM_GetCounterMode\n
  *         CR1          CMS           LL_TIM_GetCounterMode
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERMODE_UP
  *         @arg @ref LL_TIM_COUNTERMODE_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_DOWN
  *         @arg @ref LL_TIM_COUNTERMODE_CENTER_UP_DOWN
  */
/* *
  * @brief  Enable auto-reload (ARR) preload.
  * @rmtoll CR1          ARPE          LL_TIM_EnableARRPreload
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable auto-reload (ARR) preload.
  * @rmtoll CR1          ARPE          LL_TIM_DisableARRPreload
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether auto-reload (ARR) preload is enabled.
  * @rmtoll CR1          ARPE          LL_TIM_IsEnabledARRPreload
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set the division ratio between the timer clock  and the sampling clock used by the dead-time generators (when supported) and the digital filters.
  * @note Macro @ref IS_TIM_CLOCK_DIVISION_INSTANCE(TIMx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  * @rmtoll CR1          CKD           LL_TIM_SetClockDivision
  * @param  TIMx Timer instance
  * @param  ClockDivision This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  * @retval None
  */
/* *
  * @brief  Get the actual division ratio between the timer clock  and the sampling clock used by the dead-time generators (when supported) and the digital filters.
  * @note Macro @ref IS_TIM_CLOCK_DIVISION_INSTANCE(TIMx) can be used to check
  *       whether or not the clock division feature is supported by the timer
  *       instance.
  * @rmtoll CR1          CKD           LL_TIM_GetClockDivision
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV1
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV2
  *         @arg @ref LL_TIM_CLOCKDIVISION_DIV4
  */
/* *
  * @brief  Set the counter value.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @rmtoll CNT          CNT           LL_TIM_SetCounter
  * @param  TIMx Timer instance
  * @param  Counter Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  * @retval None
  */
/* *
  * @brief  Get the counter value.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @rmtoll CNT          CNT           LL_TIM_GetCounter
  * @param  TIMx Timer instance
  * @retval Counter value (between Min_Data=0 and Max_Data=0xFFFF or 0xFFFFFFFF)
  */
/* *
  * @brief  Get the current direction of the counter
  * @rmtoll CR1          DIR           LL_TIM_GetDirection
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_COUNTERDIRECTION_UP
  *         @arg @ref LL_TIM_COUNTERDIRECTION_DOWN
  */
/* *
  * @brief  Set the prescaler value.
  * @note The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1).
  * @note The prescaler can be changed on the fly as this control register is buffered. The new
  *       prescaler ratio is taken into account at the next update event.
  * @note Helper macro @ref __LL_TIM_CALC_PSC can be used to calculate the Prescaler parameter
  * @rmtoll PSC          PSC           LL_TIM_SetPrescaler
  * @param  TIMx Timer instance
  * @param  Prescaler between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Get the prescaler value.
  * @rmtoll PSC          PSC           LL_TIM_GetPrescaler
  * @param  TIMx Timer instance
  * @retval  Prescaler value between Min_Data=0 and Max_Data=65535
  */
/* *
  * @brief  Set the auto-reload value.
  * @note The counter is blocked while the auto-reload value is null.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Helper macro @ref __LL_TIM_CALC_ARR can be used to calculate the AutoReload parameter
  * @rmtoll ARR          ARR           LL_TIM_SetAutoReload
  * @param  TIMx Timer instance
  * @param  AutoReload between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Get the auto-reload value.
  * @rmtoll ARR          ARR           LL_TIM_GetAutoReload
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @param  TIMx Timer instance
  * @retval Auto-reload value
  */
/* *
  * @brief  Set the repetition counter value.
  * @note For advanced timer instances RepetitionCounter can be up to 65535.
  * @note Macro @ref IS_TIM_REPETITION_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a repetition counter.
  * @rmtoll RCR          REP           LL_TIM_SetRepetitionCounter
  * @param  TIMx Timer instance
  * @param  RepetitionCounter between Min_Data=0 and Max_Data=255
  * @retval None
  */
/* *
  * @brief  Get the repetition counter value.
  * @note Macro @ref IS_TIM_REPETITION_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a repetition counter.
  * @rmtoll RCR          REP           LL_TIM_GetRepetitionCounter
  * @param  TIMx Timer instance
  * @retval Repetition counter value
  */
/* *
  * @brief  Force a continuous copy of the update interrupt flag (UIF) into the timer counter register (bit 31).
  * @note This allows both the counter value and a potential roll-over condition signalled by the UIFCPY flag to be read in an atomic way.
  * @rmtoll CR1          UIFREMAP      LL_TIM_EnableUIFRemap
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable update interrupt flag (UIF) remapping.
  * @rmtoll CR1          UIFREMAP      LL_TIM_DisableUIFRemap
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Capture_Compare Capture Compare configuration
  * @{
  */
/* *
  * @brief  Enable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @note CCxE, CCxNE and OCxM bits are preloaded, after having been written,
  *       they are updated only when a commutation event (COM) occurs.
  * @note Only on channels that have a complementary output.
  * @note Macro @ref IS_TIM_COMMUTATION_EVENT_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @rmtoll CR2          CCPC          LL_TIM_CC_EnablePreload
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable  the capture/compare control bits (CCxE, CCxNE and OCxM) preload.
  * @note Macro @ref IS_TIM_COMMUTATION_EVENT_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @rmtoll CR2          CCPC          LL_TIM_CC_DisablePreload
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Set the updated source of the capture/compare control bits (CCxE, CCxNE and OCxM).
  * @note Macro @ref IS_TIM_COMMUTATION_EVENT_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance is able to generate a commutation event.
  * @rmtoll CR2          CCUS          LL_TIM_CC_SetUpdate
  * @param  TIMx Timer instance
  * @param  CCUpdateSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CCUPDATESOURCE_COMG_ONLY
  *         @arg @ref LL_TIM_CCUPDATESOURCE_COMG_AND_TRGI
  * @retval None
  */
/* *
  * @brief  Set the trigger of the capture/compare DMA request.
  * @rmtoll CR2          CCDS          LL_TIM_CC_SetDMAReqTrigger
  * @param  TIMx Timer instance
  * @param  DMAReqTrigger This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CCDMAREQUEST_CC
  *         @arg @ref LL_TIM_CCDMAREQUEST_UPDATE
  * @retval None
  */
/* *
  * @brief  Get actual trigger of the capture/compare DMA request.
  * @rmtoll CR2          CCDS          LL_TIM_CC_GetDMAReqTrigger
  * @param  TIMx Timer instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_CCDMAREQUEST_CC
  *         @arg @ref LL_TIM_CCDMAREQUEST_UPDATE
  */
/* *
  * @brief  Set the lock level to freeze the
  *         configuration of several capture/compare parameters.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       the lock mechanism is supported by a timer instance.
  * @rmtoll BDTR         LOCK          LL_TIM_CC_SetLockLevel
  * @param  TIMx Timer instance
  * @param  LockLevel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_LOCKLEVEL_OFF
  *         @arg @ref LL_TIM_LOCKLEVEL_1
  *         @arg @ref LL_TIM_LOCKLEVEL_2
  *         @arg @ref LL_TIM_LOCKLEVEL_3
  * @retval None
  */
/* *
  * @brief  Enable capture/compare channels.
  * @rmtoll CCER         CC1E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC1NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC2E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC2NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC3E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC3NE         LL_TIM_CC_EnableChannel\n
  *         CCER         CC4E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC5E          LL_TIM_CC_EnableChannel\n
  *         CCER         CC6E          LL_TIM_CC_EnableChannel
  * @param  TIMx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Disable capture/compare channels.
  * @rmtoll CCER         CC1E          LL_TIM_CC_DisableChannel\n
  *         CCER         CC1NE         LL_TIM_CC_DisableChannel\n
  *         CCER         CC2E          LL_TIM_CC_DisableChannel\n
  *         CCER         CC2NE         LL_TIM_CC_DisableChannel\n
  *         CCER         CC3E          LL_TIM_CC_DisableChannel\n
  *         CCER         CC3NE         LL_TIM_CC_DisableChannel\n
  *         CCER         CC4E          LL_TIM_CC_DisableChannel\n
  *         CCER         CC5E          LL_TIM_CC_DisableChannel\n
  *         CCER         CC6E          LL_TIM_CC_DisableChannel
  * @param  TIMx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Indicate whether channel(s) is(are) enabled.
  * @rmtoll CCER         CC1E          LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC1NE         LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC2E          LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC2NE         LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC3E          LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC3NE         LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC4E          LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC5E          LL_TIM_CC_IsEnabledChannel\n
  *         CCER         CC6E          LL_TIM_CC_IsEnabledChannel
  * @param  TIMx Timer instance
  * @param  Channels This parameter can be a combination of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Output_Channel Output channel configuration
  * @{
  */
/* *
  * @brief  Configure an output channel.
  * @rmtoll CCMR1        CC1S          LL_TIM_OC_ConfigOutput\n
  *         CCMR1        CC2S          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        CC3S          LL_TIM_OC_ConfigOutput\n
  *         CCMR2        CC4S          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        CC5S          LL_TIM_OC_ConfigOutput\n
  *         CCMR3        CC6S          LL_TIM_OC_ConfigOutput\n
  *         CCER         CC1P          LL_TIM_OC_ConfigOutput\n
  *         CCER         CC2P          LL_TIM_OC_ConfigOutput\n
  *         CCER         CC3P          LL_TIM_OC_ConfigOutput\n
  *         CCER         CC4P          LL_TIM_OC_ConfigOutput\n
  *         CCER         CC5P          LL_TIM_OC_ConfigOutput\n
  *         CCER         CC6P          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS1          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS2          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS3          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS4          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS5          LL_TIM_OC_ConfigOutput\n
  *         CR2          OIS6          LL_TIM_OC_ConfigOutput
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH or @ref LL_TIM_OCPOLARITY_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW or @ref LL_TIM_OCIDLESTATE_HIGH
  * @retval None
  */
/* *
  * @brief  Define the behavior of the output reference signal OCxREF from which
  *         OCx and OCxN (when relevant) are derived.
  * @rmtoll CCMR1        OC1M          LL_TIM_OC_SetMode\n
  *         CCMR1        OC2M          LL_TIM_OC_SetMode\n
  *         CCMR2        OC3M          LL_TIM_OC_SetMode\n
  *         CCMR2        OC4M          LL_TIM_OC_SetMode\n
  *         CCMR3        OC5M          LL_TIM_OC_SetMode\n
  *         CCMR3        OC6M          LL_TIM_OC_SetMode
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM1
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM1
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM2
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM1
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM2
  * @retval None
  */
/* *
  * @brief  Get the output compare mode of an output channel.
  * @rmtoll CCMR1        OC1M          LL_TIM_OC_GetMode\n
  *         CCMR1        OC2M          LL_TIM_OC_GetMode\n
  *         CCMR2        OC3M          LL_TIM_OC_GetMode\n
  *         CCMR2        OC4M          LL_TIM_OC_GetMode\n
  *         CCMR3        OC5M          LL_TIM_OC_GetMode\n
  *         CCMR3        OC6M          LL_TIM_OC_GetMode
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCMODE_FROZEN
  *         @arg @ref LL_TIM_OCMODE_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_TOGGLE
  *         @arg @ref LL_TIM_OCMODE_FORCED_INACTIVE
  *         @arg @ref LL_TIM_OCMODE_FORCED_ACTIVE
  *         @arg @ref LL_TIM_OCMODE_PWM1
  *         @arg @ref LL_TIM_OCMODE_PWM2
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM1
  *         @arg @ref LL_TIM_OCMODE_RETRIG_OPM2
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM1
  *         @arg @ref LL_TIM_OCMODE_COMBINED_PWM2
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM1
  *         @arg @ref LL_TIM_OCMODE_ASSYMETRIC_PWM2
  */
/* *
  * @brief  Set the polarity of an output channel.
  * @rmtoll CCER         CC1P          LL_TIM_OC_SetPolarity\n
  *         CCER         CC1NP         LL_TIM_OC_SetPolarity\n
  *         CCER         CC2P          LL_TIM_OC_SetPolarity\n
  *         CCER         CC2NP         LL_TIM_OC_SetPolarity\n
  *         CCER         CC3P          LL_TIM_OC_SetPolarity\n
  *         CCER         CC3NP         LL_TIM_OC_SetPolarity\n
  *         CCER         CC4P          LL_TIM_OC_SetPolarity\n
  *         CCER         CC5P          LL_TIM_OC_SetPolarity\n
  *         CCER         CC6P          LL_TIM_OC_SetPolarity
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  * @retval None
  */
/* *
  * @brief  Get the polarity of an output channel.
  * @rmtoll CCER         CC1P          LL_TIM_OC_GetPolarity\n
  *         CCER         CC1NP         LL_TIM_OC_GetPolarity\n
  *         CCER         CC2P          LL_TIM_OC_GetPolarity\n
  *         CCER         CC2NP         LL_TIM_OC_GetPolarity\n
  *         CCER         CC3P          LL_TIM_OC_GetPolarity\n
  *         CCER         CC3NP         LL_TIM_OC_GetPolarity\n
  *         CCER         CC4P          LL_TIM_OC_GetPolarity\n
  *         CCER         CC5P          LL_TIM_OC_GetPolarity\n
  *         CCER         CC6P          LL_TIM_OC_GetPolarity
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCPOLARITY_HIGH
  *         @arg @ref LL_TIM_OCPOLARITY_LOW
  */
/* *
  * @brief  Set the IDLE state of an output channel
  * @note This function is significant only for the timer instances
  *       supporting the break feature. Macro @ref IS_TIM_BREAK_INSTANCE(TIMx)
  *       can be used to check whether or not a timer instance provides
  *       a break input.
  * @rmtoll CR2         OIS1          LL_TIM_OC_SetIdleState\n
  *         CR2         OIS2N         LL_TIM_OC_SetIdleState\n
  *         CR2         OIS2          LL_TIM_OC_SetIdleState\n
  *         CR2         OIS2N         LL_TIM_OC_SetIdleState\n
  *         CR2         OIS3          LL_TIM_OC_SetIdleState\n
  *         CR2         OIS3N         LL_TIM_OC_SetIdleState\n
  *         CR2         OIS4          LL_TIM_OC_SetIdleState\n
  *         CR2         OIS5          LL_TIM_OC_SetIdleState\n
  *         CR2         OIS6          LL_TIM_OC_SetIdleState
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @param  IdleState This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_HIGH
  * @retval None
  */
/* *
  * @brief  Get the IDLE state of an output channel
  * @rmtoll CR2         OIS1          LL_TIM_OC_GetIdleState\n
  *         CR2         OIS2N         LL_TIM_OC_GetIdleState\n
  *         CR2         OIS2          LL_TIM_OC_GetIdleState\n
  *         CR2         OIS2N         LL_TIM_OC_GetIdleState\n
  *         CR2         OIS3          LL_TIM_OC_GetIdleState\n
  *         CR2         OIS3N         LL_TIM_OC_GetIdleState\n
  *         CR2         OIS4          LL_TIM_OC_GetIdleState\n
  *         CR2         OIS5          LL_TIM_OC_GetIdleState\n
  *         CR2         OIS6          LL_TIM_OC_GetIdleState
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH1N
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH2N
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH3N
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_OCIDLESTATE_LOW
  *         @arg @ref LL_TIM_OCIDLESTATE_HIGH
  */
/* *
  * @brief  Enable fast mode for the output channel.
  * @note Acts only if the channel is configured in PWM1 or PWM2 mode.
  * @rmtoll CCMR1        OC1FE          LL_TIM_OC_EnableFast\n
  *         CCMR1        OC2FE          LL_TIM_OC_EnableFast\n
  *         CCMR2        OC3FE          LL_TIM_OC_EnableFast\n
  *         CCMR2        OC4FE          LL_TIM_OC_EnableFast\n
  *         CCMR3        OC5FE          LL_TIM_OC_EnableFast\n
  *         CCMR3        OC6FE          LL_TIM_OC_EnableFast
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Disable fast mode for the output channel.
  * @rmtoll CCMR1        OC1FE          LL_TIM_OC_DisableFast\n
  *         CCMR1        OC2FE          LL_TIM_OC_DisableFast\n
  *         CCMR2        OC3FE          LL_TIM_OC_DisableFast\n
  *         CCMR2        OC4FE          LL_TIM_OC_DisableFast\n
  *         CCMR3        OC5FE          LL_TIM_OC_DisableFast\n
  *         CCMR3        OC6FE          LL_TIM_OC_DisableFast
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Indicates whether fast mode is enabled for the output channel.
  * @rmtoll CCMR1        OC1FE          LL_TIM_OC_IsEnabledFast\n
  *         CCMR1        OC2FE          LL_TIM_OC_IsEnabledFast\n
  *         CCMR2        OC3FE          LL_TIM_OC_IsEnabledFast\n
  *         CCMR2        OC4FE          LL_TIM_OC_IsEnabledFast\n
  *         CCMR3        OC5FE          LL_TIM_OC_IsEnabledFast\n
  *         CCMR3        OC6FE          LL_TIM_OC_IsEnabledFast
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable compare register (TIMx_CCRx) preload for the output channel.
  * @rmtoll CCMR1        OC1PE          LL_TIM_OC_EnablePreload\n
  *         CCMR1        OC2PE          LL_TIM_OC_EnablePreload\n
  *         CCMR2        OC3PE          LL_TIM_OC_EnablePreload\n
  *         CCMR2        OC4PE          LL_TIM_OC_EnablePreload\n
  *         CCMR3        OC5PE          LL_TIM_OC_EnablePreload\n
  *         CCMR3        OC6PE          LL_TIM_OC_EnablePreload
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Disable compare register (TIMx_CCRx) preload for the output channel.
  * @rmtoll CCMR1        OC1PE          LL_TIM_OC_DisablePreload\n
  *         CCMR1        OC2PE          LL_TIM_OC_DisablePreload\n
  *         CCMR2        OC3PE          LL_TIM_OC_DisablePreload\n
  *         CCMR2        OC4PE          LL_TIM_OC_DisablePreload\n
  *         CCMR3        OC5PE          LL_TIM_OC_DisablePreload\n
  *         CCMR3        OC6PE          LL_TIM_OC_DisablePreload
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Indicates whether compare register (TIMx_CCRx) preload is enabled for the output channel.
  * @rmtoll CCMR1        OC1PE          LL_TIM_OC_IsEnabledPreload\n
  *         CCMR1        OC2PE          LL_TIM_OC_IsEnabledPreload\n
  *         CCMR2        OC3PE          LL_TIM_OC_IsEnabledPreload\n
  *         CCMR2        OC4PE          LL_TIM_OC_IsEnabledPreload\n
  *         CCMR3        OC5PE          LL_TIM_OC_IsEnabledPreload\n
  *         CCMR3        OC6PE          LL_TIM_OC_IsEnabledPreload
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable clearing the output channel on an external event.
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro @ref IS_TIM_OCXREF_CLEAR_INSTANCE(TIMx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @rmtoll CCMR1        OC1CE          LL_TIM_OC_EnableClear\n
  *         CCMR1        OC2CE          LL_TIM_OC_EnableClear\n
  *         CCMR2        OC3CE          LL_TIM_OC_EnableClear\n
  *         CCMR2        OC4CE          LL_TIM_OC_EnableClear\n
  *         CCMR3        OC5CE          LL_TIM_OC_EnableClear\n
  *         CCMR3        OC6CE          LL_TIM_OC_EnableClear
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Disable clearing the output channel on an external event.
  * @note Macro @ref IS_TIM_OCXREF_CLEAR_INSTANCE(TIMx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @rmtoll CCMR1        OC1CE          LL_TIM_OC_DisableClear\n
  *         CCMR1        OC2CE          LL_TIM_OC_DisableClear\n
  *         CCMR2        OC3CE          LL_TIM_OC_DisableClear\n
  *         CCMR2        OC4CE          LL_TIM_OC_DisableClear\n
  *         CCMR3        OC5CE          LL_TIM_OC_DisableClear\n
  *         CCMR3        OC6CE          LL_TIM_OC_DisableClear
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval None
  */
/* *
  * @brief  Indicates clearing the output channel on an external event is enabled for the output channel.
  * @note This function enables clearing the output channel on an external event.
  * @note This function can only be used in Output compare and PWM modes. It does not work in Forced mode.
  * @note Macro @ref IS_TIM_OCXREF_CLEAR_INSTANCE(TIMx) can be used to check whether
  *       or not a timer instance can clear the OCxREF signal on an external event.
  * @rmtoll CCMR1        OC1CE          LL_TIM_OC_IsEnabledClear\n
  *         CCMR1        OC2CE          LL_TIM_OC_IsEnabledClear\n
  *         CCMR2        OC3CE          LL_TIM_OC_IsEnabledClear\n
  *         CCMR2        OC4CE          LL_TIM_OC_IsEnabledClear\n
  *         CCMR3        OC5CE          LL_TIM_OC_IsEnabledClear\n
  *         CCMR3        OC6CE          LL_TIM_OC_IsEnabledClear
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  *         @arg @ref LL_TIM_CHANNEL_CH5
  *         @arg @ref LL_TIM_CHANNEL_CH6
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set the dead-time delay (delay inserted between the rising edge of the OCxREF signal and the rising edge if the Ocx and OCxN signals).
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       dead-time insertion feature is supported by a timer instance.
  * @note Helper macro @ref __LL_TIM_CALC_DEADTIME can be used to calculate the DeadTime parameter
  * @rmtoll BDTR         DTG           LL_TIM_OC_SetDeadTime
  * @param  TIMx Timer instance
  * @param  DeadTime between Min_Data=0 and Max_Data=255
  * @retval None
  */
/* *
  * @brief  Set compare value for output channel 1 (TIMx_CCR1).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC1_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 1 is supported by a timer instance.
  * @rmtoll CCR1         CCR1          LL_TIM_OC_SetCompareCH1
  * @param  TIMx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Set compare value for output channel 2 (TIMx_CCR2).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC2_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 2 is supported by a timer instance.
  * @rmtoll CCR2         CCR2          LL_TIM_OC_SetCompareCH2
  * @param  TIMx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Set compare value for output channel 3 (TIMx_CCR3).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  *       output channel is supported by a timer instance.
  * @rmtoll CCR3         CCR3          LL_TIM_OC_SetCompareCH3
  * @param  TIMx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Set compare value for output channel 4 (TIMx_CCR4).
  * @note In 32-bit timer implementations compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 4 is supported by a timer instance.
  * @rmtoll CCR4         CCR4          LL_TIM_OC_SetCompareCH4
  * @param  TIMx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Set compare value for output channel 5 (TIMx_CCR5).
  * @note Macro @ref IS_TIM_CC5_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 5 is supported by a timer instance.
  * @rmtoll CCR5         CCR5          LL_TIM_OC_SetCompareCH5
  * @param  TIMx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Set compare value for output channel 6 (TIMx_CCR6).
  * @note Macro @ref IS_TIM_CC6_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 6 is supported by a timer instance.
  * @rmtoll CCR6         CCR6          LL_TIM_OC_SetCompareCH6
  * @param  TIMx Timer instance
  * @param  CompareValue between Min_Data=0 and Max_Data=65535
  * @retval None
  */
/* *
  * @brief  Get compare value (TIMx_CCR1) set for  output channel 1.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC1_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 1 is supported by a timer instance.
  * @rmtoll CCR1         CCR1          LL_TIM_OC_GetCompareCH1
  * @param  TIMx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get compare value (TIMx_CCR2) set for  output channel 2.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC2_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 2 is supported by a timer instance.
  * @rmtoll CCR2         CCR2          LL_TIM_OC_GetCompareCH2
  * @param  TIMx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get compare value (TIMx_CCR3) set for  output channel 3.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 3 is supported by a timer instance.
  * @rmtoll CCR3         CCR3          LL_TIM_OC_GetCompareCH3
  * @param  TIMx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get compare value (TIMx_CCR4) set for  output channel 4.
  * @note In 32-bit timer implementations returned compare value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 4 is supported by a timer instance.
  * @rmtoll CCR4         CCR4          LL_TIM_OC_GetCompareCH4
  * @param  TIMx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get compare value (TIMx_CCR5) set for  output channel 5.
  * @note Macro @ref IS_TIM_CC5_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 5 is supported by a timer instance.
  * @rmtoll CCR5         CCR5          LL_TIM_OC_GetCompareCH5
  * @param  TIMx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get compare value (TIMx_CCR6) set for  output channel 6.
  * @note Macro @ref IS_TIM_CC6_INSTANCE(TIMx) can be used to check whether or not
  *       output channel 6 is supported by a timer instance.
  * @rmtoll CCR6         CCR6          LL_TIM_OC_GetCompareCH6
  * @param  TIMx Timer instance
  * @retval CompareValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Select on which reference signal the OC5REF is combined to.
  * @note Macro @ref IS_TIM_COMBINED3PHASEPWM_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports the combined 3-phase PWM mode.
  * @rmtoll CCR5         GC5C3          LL_TIM_SetCH5CombinedChannels\n
  *         CCR5         GC5C2          LL_TIM_SetCH5CombinedChannels\n
  *         CCR5         GC5C1          LL_TIM_SetCH5CombinedChannels
  * @param  TIMx Timer instance
  * @param  GroupCH5 This parameter can be one of the following values:
  *         @arg @ref LL_TIM_GROUPCH5_NONE
  *         @arg @ref LL_TIM_GROUPCH5_OC1REFC
  *         @arg @ref LL_TIM_GROUPCH5_OC2REFC
  *         @arg @ref LL_TIM_GROUPCH5_OC3REFC
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Input_Channel Input channel configuration
  * @{
  */
/* *
  * @brief  Configure input channel.
  * @rmtoll CCMR1        CC1S          LL_TIM_IC_Config\n
  *         CCMR1        IC1PSC        LL_TIM_IC_Config\n
  *         CCMR1        IC1F          LL_TIM_IC_Config\n
  *         CCMR1        CC2S          LL_TIM_IC_Config\n
  *         CCMR1        IC2PSC        LL_TIM_IC_Config\n
  *         CCMR1        IC2F          LL_TIM_IC_Config\n
  *         CCMR2        CC3S          LL_TIM_IC_Config\n
  *         CCMR2        IC3PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC3F          LL_TIM_IC_Config\n
  *         CCMR2        CC4S          LL_TIM_IC_Config\n
  *         CCMR2        IC4PSC        LL_TIM_IC_Config\n
  *         CCMR2        IC4F          LL_TIM_IC_Config\n
  *         CCER         CC1P          LL_TIM_IC_Config\n
  *         CCER         CC1NP         LL_TIM_IC_Config\n
  *         CCER         CC2P          LL_TIM_IC_Config\n
  *         CCER         CC2NP         LL_TIM_IC_Config\n
  *         CCER         CC3P          LL_TIM_IC_Config\n
  *         CCER         CC3NP         LL_TIM_IC_Config\n
  *         CCER         CC4P          LL_TIM_IC_Config\n
  *         CCER         CC4NP         LL_TIM_IC_Config
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  Configuration This parameter must be a combination of all the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECTTI or @ref LL_TIM_ACTIVEINPUT_INDIRECTTI or @ref LL_TIM_ACTIVEINPUT_TRC
  *         @arg @ref LL_TIM_ICPSC_DIV1 or ... or @ref LL_TIM_ICPSC_DIV8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1 or ... or @ref LL_TIM_IC_FILTER_FDIV32_N8
  *         @arg @ref LL_TIM_IC_POLARITY_RISING or @ref LL_TIM_IC_POLARITY_FALLING or @ref LL_TIM_IC_POLARITY_BOTHEDGE
  * @retval None
  */
/* *
  * @brief  Set the active input.
  * @rmtoll CCMR1        CC1S          LL_TIM_IC_SetActiveInput\n
  *         CCMR1        CC2S          LL_TIM_IC_SetActiveInput\n
  *         CCMR2        CC3S          LL_TIM_IC_SetActiveInput\n
  *         CCMR2        CC4S          LL_TIM_IC_SetActiveInput
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICActiveInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_INDIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_TRC
  * @retval None
  */
/* *
  * @brief  Get the current active input.
  * @rmtoll CCMR1        CC1S          LL_TIM_IC_GetActiveInput\n
  *         CCMR1        CC2S          LL_TIM_IC_GetActiveInput\n
  *         CCMR2        CC3S          LL_TIM_IC_GetActiveInput\n
  *         CCMR2        CC4S          LL_TIM_IC_GetActiveInput
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ACTIVEINPUT_DIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_INDIRECTTI
  *         @arg @ref LL_TIM_ACTIVEINPUT_TRC
  */
/* *
  * @brief  Set the prescaler of input channel.
  * @rmtoll CCMR1        IC1PSC        LL_TIM_IC_SetPrescaler\n
  *         CCMR1        IC2PSC        LL_TIM_IC_SetPrescaler\n
  *         CCMR2        IC3PSC        LL_TIM_IC_SetPrescaler\n
  *         CCMR2        IC4PSC        LL_TIM_IC_SetPrescaler
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICPrescaler This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  * @retval None
  */
/* *
  * @brief  Get the current prescaler value acting on an  input channel.
  * @rmtoll CCMR1        IC1PSC        LL_TIM_IC_GetPrescaler\n
  *         CCMR1        IC2PSC        LL_TIM_IC_GetPrescaler\n
  *         CCMR2        IC3PSC        LL_TIM_IC_GetPrescaler\n
  *         CCMR2        IC4PSC        LL_TIM_IC_GetPrescaler
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_ICPSC_DIV1
  *         @arg @ref LL_TIM_ICPSC_DIV2
  *         @arg @ref LL_TIM_ICPSC_DIV4
  *         @arg @ref LL_TIM_ICPSC_DIV8
  */
/* *
  * @brief  Set the input filter duration.
  * @rmtoll CCMR1        IC1F          LL_TIM_IC_SetFilter\n
  *         CCMR1        IC2F          LL_TIM_IC_SetFilter\n
  *         CCMR2        IC3F          LL_TIM_IC_SetFilter\n
  *         CCMR2        IC4F          LL_TIM_IC_SetFilter
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICFilter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N8
  * @retval None
  */
/* *
  * @brief  Get the input filter duration.
  * @rmtoll CCMR1        IC1F          LL_TIM_IC_GetFilter\n
  *         CCMR1        IC2F          LL_TIM_IC_GetFilter\n
  *         CCMR2        IC3F          LL_TIM_IC_GetFilter\n
  *         CCMR2        IC4F          LL_TIM_IC_GetFilter
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_IC_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_IC_FILTER_FDIV32_N8
  */
/* *
  * @brief  Set the input channel polarity.
  * @rmtoll CCER         CC1P          LL_TIM_IC_SetPolarity\n
  *         CCER         CC1NP         LL_TIM_IC_SetPolarity\n
  *         CCER         CC2P          LL_TIM_IC_SetPolarity\n
  *         CCER         CC2NP         LL_TIM_IC_SetPolarity\n
  *         CCER         CC3P          LL_TIM_IC_SetPolarity\n
  *         CCER         CC3NP         LL_TIM_IC_SetPolarity\n
  *         CCER         CC4P          LL_TIM_IC_SetPolarity\n
  *         CCER         CC4NP         LL_TIM_IC_SetPolarity
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @param  ICPolarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_IC_POLARITY_RISING
  *         @arg @ref LL_TIM_IC_POLARITY_FALLING
  *         @arg @ref LL_TIM_IC_POLARITY_BOTHEDGE
  * @retval None
  */
/* *
  * @brief  Get the current input channel polarity.
  * @rmtoll CCER         CC1P          LL_TIM_IC_GetPolarity\n
  *         CCER         CC1NP         LL_TIM_IC_GetPolarity\n
  *         CCER         CC2P          LL_TIM_IC_GetPolarity\n
  *         CCER         CC2NP         LL_TIM_IC_GetPolarity\n
  *         CCER         CC3P          LL_TIM_IC_GetPolarity\n
  *         CCER         CC3NP         LL_TIM_IC_GetPolarity\n
  *         CCER         CC4P          LL_TIM_IC_GetPolarity\n
  *         CCER         CC4NP         LL_TIM_IC_GetPolarity
  * @param  TIMx Timer instance
  * @param  Channel This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CHANNEL_CH1
  *         @arg @ref LL_TIM_CHANNEL_CH2
  *         @arg @ref LL_TIM_CHANNEL_CH3
  *         @arg @ref LL_TIM_CHANNEL_CH4
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_TIM_IC_POLARITY_RISING
  *         @arg @ref LL_TIM_IC_POLARITY_FALLING
  *         @arg @ref LL_TIM_IC_POLARITY_BOTHEDGE
  */
/* *
  * @brief  Connect the TIMx_CH1, CH2 and CH3 pins  to the TI1 input (XOR combination).
  * @note Macro @ref IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  * @rmtoll CR2          TI1S          LL_TIM_IC_EnableXORCombination
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disconnect the TIMx_CH1, CH2 and CH3 pins  from the TI1 input.
  * @note Macro @ref IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides an XOR input.
  * @rmtoll CR2          TI1S          LL_TIM_IC_DisableXORCombination
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the TIMx_CH1, CH2 and CH3 pins are connectected to the TI1 input.
  * @note Macro @ref IS_TIM_XOR_INSTANCE(TIMx) can be used to check whether or not
  * a timer instance provides an XOR input.
  * @rmtoll CR2          TI1S          LL_TIM_IC_IsEnabledXORCombination
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get captured value for input channel 1.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC1_INSTANCE(TIMx) can be used to check whether or not
  *       input channel 1 is supported by a timer instance.
  * @rmtoll CCR1         CCR1          LL_TIM_IC_GetCaptureCH1
  * @param  TIMx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get captured value for input channel 2.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC2_INSTANCE(TIMx) can be used to check whether or not
  *       input channel 2 is supported by a timer instance.
  * @rmtoll CCR2         CCR2          LL_TIM_IC_GetCaptureCH2
  * @param  TIMx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get captured value for input channel 3.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC3_INSTANCE(TIMx) can be used to check whether or not
  *       input channel 3 is supported by a timer instance.
  * @rmtoll CCR3         CCR3          LL_TIM_IC_GetCaptureCH3
  * @param  TIMx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @brief  Get captured value for input channel 4.
  * @note In 32-bit timer implementations returned captured value can be between 0x00000000 and 0xFFFFFFFF.
  * @note Macro @ref IS_TIM_32B_COUNTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports a 32 bits counter.
  * @note Macro @ref IS_TIM_CC4_INSTANCE(TIMx) can be used to check whether or not
  *       input channel 4 is supported by a timer instance.
  * @rmtoll CCR4         CCR4          LL_TIM_IC_GetCaptureCH4
  * @param  TIMx Timer instance
  * @retval CapturedValue (between Min_Data=0 and Max_Data=65535)
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Clock_Selection Counter clock selection
  * @{
  */
/* *
  * @brief  Enable external clock mode 2.
  * @note When external clock mode 2 is enabled the counter is clocked by any active edge on the ETRF signal.
  * @note Macro @ref IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @rmtoll SMCR         ECE           LL_TIM_EnableExternalClock
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable external clock mode 2.
  * @note Macro @ref IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @rmtoll SMCR         ECE           LL_TIM_DisableExternalClock
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether external clock mode 2 is enabled.
  * @note Macro @ref IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @rmtoll SMCR         ECE           LL_TIM_IsEnabledExternalClock
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set the clock source of the counter clock.
  * @note when selected clock source is external clock mode 1, the timer input
  *       the external clock is applied is selected by calling the @ref LL_TIM_SetTriggerInput()
  *       function. This timer input must be configured by calling
  *       the @ref LL_TIM_IC_Config() function.
  * @note Macro @ref IS_TIM_CLOCKSOURCE_ETRMODE1_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports external clock mode1.
  * @note Macro @ref IS_TIM_CLOCKSOURCE_ETRMODE2_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports external clock mode2.
  * @rmtoll SMCR         SMS           LL_TIM_SetClockSource\n
  *         SMCR         ECE           LL_TIM_SetClockSource
  * @param  TIMx Timer instance
  * @param  ClockSource This parameter can be one of the following values:
  *         @arg @ref LL_TIM_CLOCKSOURCE_INTERNAL
  *         @arg @ref LL_TIM_CLOCKSOURCE_EXT_MODE1
  *         @arg @ref LL_TIM_CLOCKSOURCE_EXT_MODE2
  * @retval None
  */
/* *
  * @brief  Set the encoder interface mode.
  * @note Macro @ref IS_TIM_ENCODER_INTERFACE_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance supports the encoder mode.
  * @rmtoll SMCR         SMS           LL_TIM_SetEncoderMode
  * @param  TIMx Timer instance
  * @param  EncoderMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ENCODERMODE_X2_TI1
  *         @arg @ref LL_TIM_ENCODERMODE_X2_TI2
  *         @arg @ref LL_TIM_ENCODERMODE_X4_TI12
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Timer_Synchronization Timer synchronisation configuration
  * @{
  */
/* *
  * @brief  Set the trigger output (TRGO) used for timer synchronization .
  * @note Macro @ref IS_TIM_MASTER_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance can operate as a master timer.
  * @rmtoll CR2          MMS           LL_TIM_SetTriggerOutput
  * @param  TIMx Timer instance
  * @param  TimerSynchronization This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TRGO_RESET
  *         @arg @ref LL_TIM_TRGO_ENABLE
  *         @arg @ref LL_TIM_TRGO_UPDATE
  *         @arg @ref LL_TIM_TRGO_CC1IF
  *         @arg @ref LL_TIM_TRGO_OC1REF
  *         @arg @ref LL_TIM_TRGO_OC2REF
  *         @arg @ref LL_TIM_TRGO_OC3REF
  *         @arg @ref LL_TIM_TRGO_OC4REF
  * @retval None
  */
/* *
  * @brief  Set the trigger output 2 (TRGO2) used for ADC synchronization .
  * @note Macro @ref IS_TIM_TRGO2_INSTANCE(TIMx) can be used to check
  *       whether or not a timer instance can be used for ADC synchronization.
  * @rmtoll CR2          MMS2          LL_TIM_SetTriggerOutput2
  * @param  TIMx Timer Instance
  * @param  ADCSynchronization This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TRGO2_RESET
  *         @arg @ref LL_TIM_TRGO2_ENABLE
  *         @arg @ref LL_TIM_TRGO2_UPDATE
  *         @arg @ref LL_TIM_TRGO2_CC1F
  *         @arg @ref LL_TIM_TRGO2_OC1
  *         @arg @ref LL_TIM_TRGO2_OC2
  *         @arg @ref LL_TIM_TRGO2_OC3
  *         @arg @ref LL_TIM_TRGO2_OC4
  *         @arg @ref LL_TIM_TRGO2_OC5
  *         @arg @ref LL_TIM_TRGO2_OC6
  *         @arg @ref LL_TIM_TRGO2_OC4_RISINGFALLING
  *         @arg @ref LL_TIM_TRGO2_OC6_RISINGFALLING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC4_RISING_OC6_FALLING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_RISING
  *         @arg @ref LL_TIM_TRGO2_OC5_RISING_OC6_FALLING
  * @retval None
  */
/* *
  * @brief  Set the synchronization mode of a slave timer.
  * @note Macro @ref IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @rmtoll SMCR         SMS           LL_TIM_SetSlaveMode
  * @param  TIMx Timer instance
  * @param  SlaveMode This parameter can be one of the following values:
  *         @arg @ref LL_TIM_SLAVEMODE_DISABLED
  *         @arg @ref LL_TIM_SLAVEMODE_RESET
  *         @arg @ref LL_TIM_SLAVEMODE_GATED
  *         @arg @ref LL_TIM_SLAVEMODE_TRIGGER
  *         @arg @ref LL_TIM_SLAVEMODE_COMBINED_RESETTRIGGER
  * @retval None
  */
/* *
  * @brief  Set the selects the trigger input to be used to synchronize the counter.
  * @note Macro @ref IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @rmtoll SMCR         TS            LL_TIM_SetTriggerInput
  * @param  TIMx Timer instance
  * @param  TriggerInput This parameter can be one of the following values:
  *         @arg @ref LL_TIM_TS_ITR0
  *         @arg @ref LL_TIM_TS_ITR1
  *         @arg @ref LL_TIM_TS_ITR2
  *         @arg @ref LL_TIM_TS_ITR3
  *         @arg @ref LL_TIM_TS_TI1F_ED
  *         @arg @ref LL_TIM_TS_TI1FP1
  *         @arg @ref LL_TIM_TS_TI2FP2
  *         @arg @ref LL_TIM_TS_ETRF
  * @retval None
  */
/* *
  * @brief  Enable the Master/Slave mode.
  * @note Macro @ref IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @rmtoll SMCR         MSM           LL_TIM_EnableMasterSlaveMode
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable the Master/Slave mode.
  * @note Macro @ref IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance can operate as a slave timer.
  * @rmtoll SMCR         MSM           LL_TIM_DisableMasterSlaveMode
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief Indicates whether the Master/Slave mode is enabled.
  * @note Macro @ref IS_TIM_SLAVE_INSTANCE(TIMx) can be used to check whether or not
  * a timer instance can operate as a slave timer.
  * @rmtoll SMCR         MSM           LL_TIM_IsEnabledMasterSlaveMode
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Configure the external trigger (ETR) input.
  * @note Macro @ref IS_TIM_ETR_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides an external trigger input.
  * @rmtoll SMCR         ETP           LL_TIM_ConfigETR\n
  *         SMCR         ETPS          LL_TIM_ConfigETR\n
  *         SMCR         ETF           LL_TIM_ConfigETR
  * @param  TIMx Timer instance
  * @param  ETRPolarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETR_POLARITY_NONINVERTED
  *         @arg @ref LL_TIM_ETR_POLARITY_INVERTED
  * @param  ETRPrescaler This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV1
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV2
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV4
  *         @arg @ref LL_TIM_ETR_PRESCALER_DIV8
  * @param  ETRFilter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_ETR_FILTER_FDIV32_N8
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Break_Function Break function configuration
  * @{
  */
/* *
  * @brief  Enable the break function.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         BKE           LL_TIM_EnableBRK
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable the break function.
  * @rmtoll BDTR         BKE           LL_TIM_DisableBRK
  * @param  TIMx Timer instance
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @retval None
  */
/* *
  * @brief  Configure the break input.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         BKP           LL_TIM_ConfigBRK\n
  *         BDTR         BKF           LL_TIM_ConfigBRK
  * @param  TIMx Timer instance
  * @param  BreakPolarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK_POLARITY_HIGH
  * @param  BreakFilter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK_FILTER_FDIV32_N8
  * @retval None
  */
/* *
  * @brief  Enable the break 2 function.
  * @note Macro @ref IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a second break input.
  * @rmtoll BDTR         BK2E          LL_TIM_EnableBRK2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable the break  2 function.
  * @note Macro @ref IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a second break input.
  * @rmtoll BDTR         BK2E          LL_TIM_DisableBRK2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Configure the break 2 input.
  * @note Macro @ref IS_TIM_BKIN2_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a second break input.
  * @rmtoll BDTR         BK2P          LL_TIM_ConfigBRK2\n
  *         BDTR         BK2F          LL_TIM_ConfigBRK2
  * @param  TIMx Timer instance
  * @param  Break2Polarity This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_POLARITY_LOW
  *         @arg @ref LL_TIM_BREAK2_POLARITY_HIGH
  * @param  Break2Filter This parameter can be one of the following values:
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N2
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N4
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV1_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV2_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV4_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV8_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV16_N8
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N5
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N6
  *         @arg @ref LL_TIM_BREAK2_FILTER_FDIV32_N8
  * @retval None
  */
/* *
  * @brief  Select the outputs off state (enabled v.s. disabled) in Idle and Run modes.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         OSSI          LL_TIM_SetOffStates\n
  *         BDTR         OSSR          LL_TIM_SetOffStates
  * @param  TIMx Timer instance
  * @param  OffStateIdle This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OSSI_DISABLE
  *         @arg @ref LL_TIM_OSSI_ENABLE
  * @param  OffStateRun This parameter can be one of the following values:
  *         @arg @ref LL_TIM_OSSR_DISABLE
  *         @arg @ref LL_TIM_OSSR_ENABLE
  * @retval None
  */
/* *
  * @brief  Enable automatic output (MOE can be set by software or automatically when a break input is active).
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         AOE           LL_TIM_EnableAutomaticOutput
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable automatic output (MOE can be set only by software).
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         AOE           LL_TIM_DisableAutomaticOutput
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether automatic output is enabled.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         AOE           LL_TIM_IsEnabledAutomaticOutput
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable the outputs (set the MOE bit in TIMx_BDTR register).
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_EnableAllOutputs
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable the outputs (reset the MOE bit in TIMx_BDTR register).
  * @note The MOE bit in TIMx_BDTR register allows to enable /disable the outputs by
  *       software and is reset in case of break or break2 event.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_DisableAllOutputs
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether outputs are enabled.
  * @note Macro @ref IS_TIM_BREAK_INSTANCE(TIMx) can be used to check whether or not
  *       a timer instance provides a break input.
  * @rmtoll BDTR         MOE           LL_TIM_IsEnabledAllOutputs
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* TIM_BREAK_INPUT_SUPPORT */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_DMA_Burst_Mode DMA burst mode configuration
  * @{
  */
/* *
  * @brief  Configures the timer DMA burst feature.
  * @note Macro @ref IS_TIM_DMABURST_INSTANCE(TIMx) can be used to check whether or
  *       not a timer instance supports the DMA burst mode.
  * @rmtoll DCR          DBL           LL_TIM_ConfigDMABurst\n
  *         DCR          DBA           LL_TIM_ConfigDMABurst
  * @param  TIMx Timer instance
  * @param  DMABurstBaseAddress This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SMCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_DIER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_SR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_EGR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCER
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CNT
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_PSC
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_ARR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_RCR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR2
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR4
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_BDTR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCMR3
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR5
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_CCR6
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_OR
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF1
  *         @arg @ref LL_TIM_DMABURST_BASEADDR_AF2
  * @param  DMABurstLength This parameter can be one of the following values:
  *         @arg @ref LL_TIM_DMABURST_LENGTH_1TRANSFER
  *         @arg @ref LL_TIM_DMABURST_LENGTH_2TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_3TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_4TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_5TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_6TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_7TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_8TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_9TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_10TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_11TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_12TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_13TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_14TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_15TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_16TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_17TRANSFERS
  *         @arg @ref LL_TIM_DMABURST_LENGTH_18TRANSFERS
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_Timer_Inputs_Remapping Timer input remapping
  * @{
  */
/* *
  * @brief  Remap TIM inputs (input channel, internal/external triggers).
  * @note Macro @ref IS_TIM_REMAP_INSTANCE(TIMx) can be used to check whether or not
  *       a some timer inputs can be remapped.
  * @rmtoll TIM2_OR     ITR1_RMP          LL_TIM_SetRemap\n
  *         TIM5_OR     TI4_RMP           LL_TIM_SetRemap\n
  *         TIM11_OR    TI1_RMP           LL_TIM_SetRemap
  * @param  TIMx Timer instance
  * @param  Remap Remap param depends on the TIMx. Description available only
  *         in CHM version of the User Manual (not in .pdf).
  *         Otherwise see Reference Manual description of OR registers.
  *
  *         Below description summarizes "Timer Instance" and "Remap" param combinations:
  *
  *         TIM2: one of the following values
  *
  *            ITR1_RMP can be one of the following values
  *            @arg @ref LL_TIM_TIM2_ITR1_RMP_TIM8_TRGO
  *            @arg @ref LL_TIM_TIM2_ITR1_RMP_ETH_PTP
  *            @arg @ref LL_TIM_TIM2_ITR1_RMP_OTG_FS_SOF
  *            @arg @ref LL_TIM_TIM2_ITR1_RMP_OTG_HS_SOF
  *
  *         TIM5: one of the following values
  *
  *            @arg @ref LL_TIM_TIM5_TI4_RMP_GPIO
  *            @arg @ref LL_TIM_TIM5_TI4_RMP_LSI
  *            @arg @ref LL_TIM_TIM5_TI4_RMP_LSE
  *            @arg @ref LL_TIM_TIM5_TI4_RMP_RTC
  *
  *         TIM11: one of the following values
  *
  *            @arg @ref LL_TIM_TIM11_TI1_RMP_GPIO
  *            @arg @ref LL_TIM_TIM11_TI1_RMP_SPDIFRX
  *            @arg @ref LL_TIM_TIM11_TI1_RMP_HSE
  *            @arg @ref LL_TIM_TIM11_TI1_RMP_MCO1
  *
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_FLAG_Management FLAG-Management
  * @{
  */
/* *
  * @brief  Clear the update interrupt flag (UIF).
  * @rmtoll SR           UIF           LL_TIM_ClearFlag_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether update interrupt flag (UIF) is set (update interrupt is pending).
  * @rmtoll SR           UIF           LL_TIM_IsActiveFlag_UPDATE
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 1 interrupt flag (CC1F).
  * @rmtoll SR           CC1IF         LL_TIM_ClearFlag_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 1 interrupt flag (CC1F) is set (Capture/Compare 1 interrupt is pending).
  * @rmtoll SR           CC1IF         LL_TIM_IsActiveFlag_CC1
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 2 interrupt flag (CC2F).
  * @rmtoll SR           CC2IF         LL_TIM_ClearFlag_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 2 interrupt flag (CC2F) is set (Capture/Compare 2 interrupt is pending).
  * @rmtoll SR           CC2IF         LL_TIM_IsActiveFlag_CC2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 3 interrupt flag (CC3F).
  * @rmtoll SR           CC3IF         LL_TIM_ClearFlag_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 3 interrupt flag (CC3F) is set (Capture/Compare 3 interrupt is pending).
  * @rmtoll SR           CC3IF         LL_TIM_IsActiveFlag_CC3
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 4 interrupt flag (CC4F).
  * @rmtoll SR           CC4IF         LL_TIM_ClearFlag_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 4 interrupt flag (CC4F) is set (Capture/Compare 4 interrupt is pending).
  * @rmtoll SR           CC4IF         LL_TIM_IsActiveFlag_CC4
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 5 interrupt flag (CC5F).
  * @rmtoll SR           CC5IF         LL_TIM_ClearFlag_CC5
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 5 interrupt flag (CC5F) is set (Capture/Compare 5 interrupt is pending).
  * @rmtoll SR           CC5IF         LL_TIM_IsActiveFlag_CC5
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 6 interrupt flag (CC6F).
  * @rmtoll SR           CC6IF         LL_TIM_ClearFlag_CC6
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 6 interrupt flag (CC6F) is set (Capture/Compare 6 interrupt is pending).
  * @rmtoll SR           CC6IF         LL_TIM_IsActiveFlag_CC6
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the commutation interrupt flag (COMIF).
  * @rmtoll SR           COMIF         LL_TIM_ClearFlag_COM
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether commutation interrupt flag (COMIF) is set (commutation interrupt is pending).
  * @rmtoll SR           COMIF         LL_TIM_IsActiveFlag_COM
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the trigger interrupt flag (TIF).
  * @rmtoll SR           TIF           LL_TIM_ClearFlag_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether trigger interrupt flag (TIF) is set (trigger interrupt is pending).
  * @rmtoll SR           TIF           LL_TIM_IsActiveFlag_TRIG
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the break interrupt flag (BIF).
  * @rmtoll SR           BIF           LL_TIM_ClearFlag_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether break interrupt flag (BIF) is set (break interrupt is pending).
  * @rmtoll SR           BIF           LL_TIM_IsActiveFlag_BRK
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the break 2 interrupt flag (B2IF).
  * @rmtoll SR           B2IF          LL_TIM_ClearFlag_BRK2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether break 2 interrupt flag (B2IF) is set (break 2 interrupt is pending).
  * @rmtoll SR           B2IF          LL_TIM_IsActiveFlag_BRK2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 1 over-capture interrupt flag (CC1OF).
  * @rmtoll SR           CC1OF         LL_TIM_ClearFlag_CC1OVR
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 1 over-capture interrupt flag (CC1OF) is set (Capture/Compare 1 interrupt is pending).
  * @rmtoll SR           CC1OF         LL_TIM_IsActiveFlag_CC1OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 2 over-capture interrupt flag (CC2OF).
  * @rmtoll SR           CC2OF         LL_TIM_ClearFlag_CC2OVR
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 2 over-capture interrupt flag (CC2OF) is set (Capture/Compare 2 over-capture interrupt is pending).
  * @rmtoll SR           CC2OF         LL_TIM_IsActiveFlag_CC2OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 3 over-capture interrupt flag (CC3OF).
  * @rmtoll SR           CC3OF         LL_TIM_ClearFlag_CC3OVR
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 3 over-capture interrupt flag (CC3OF) is set (Capture/Compare 3 over-capture interrupt is pending).
  * @rmtoll SR           CC3OF         LL_TIM_IsActiveFlag_CC3OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the Capture/Compare 4 over-capture interrupt flag (CC4OF).
  * @rmtoll SR           CC4OF         LL_TIM_ClearFlag_CC4OVR
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether Capture/Compare 4 over-capture interrupt flag (CC4OF) is set (Capture/Compare 4 over-capture interrupt is pending).
  * @rmtoll SR           CC4OF         LL_TIM_IsActiveFlag_CC4OVR
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear the system break interrupt flag (SBIF).
  * @rmtoll SR           SBIF          LL_TIM_ClearFlag_SYSBRK
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicate whether system break interrupt flag (SBIF) is set (system break interrupt is pending).
  * @rmtoll SR           SBIF          LL_TIM_IsActiveFlag_SYSBRK
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_IT_Management IT-Management
  * @{
  */
/* *
  * @brief  Enable update interrupt (UIE).
  * @rmtoll DIER         UIE           LL_TIM_EnableIT_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable update interrupt (UIE).
  * @rmtoll DIER         UIE           LL_TIM_DisableIT_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the update interrupt (UIE) is enabled.
  * @rmtoll DIER         UIE           LL_TIM_IsEnabledIT_UPDATE
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 1 interrupt (CC1IE).
  * @rmtoll DIER         CC1IE         LL_TIM_EnableIT_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 1  interrupt (CC1IE).
  * @rmtoll DIER         CC1IE         LL_TIM_DisableIT_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 1 interrupt (CC1IE) is enabled.
  * @rmtoll DIER         CC1IE         LL_TIM_IsEnabledIT_CC1
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 2 interrupt (CC2IE).
  * @rmtoll DIER         CC2IE         LL_TIM_EnableIT_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 2  interrupt (CC2IE).
  * @rmtoll DIER         CC2IE         LL_TIM_DisableIT_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 2 interrupt (CC2IE) is enabled.
  * @rmtoll DIER         CC2IE         LL_TIM_IsEnabledIT_CC2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 3 interrupt (CC3IE).
  * @rmtoll DIER         CC3IE         LL_TIM_EnableIT_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 3  interrupt (CC3IE).
  * @rmtoll DIER         CC3IE         LL_TIM_DisableIT_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 3 interrupt (CC3IE) is enabled.
  * @rmtoll DIER         CC3IE         LL_TIM_IsEnabledIT_CC3
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 4 interrupt (CC4IE).
  * @rmtoll DIER         CC4IE         LL_TIM_EnableIT_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 4  interrupt (CC4IE).
  * @rmtoll DIER         CC4IE         LL_TIM_DisableIT_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 4 interrupt (CC4IE) is enabled.
  * @rmtoll DIER         CC4IE         LL_TIM_IsEnabledIT_CC4
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable commutation interrupt (COMIE).
  * @rmtoll DIER         COMIE         LL_TIM_EnableIT_COM
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable commutation interrupt (COMIE).
  * @rmtoll DIER         COMIE         LL_TIM_DisableIT_COM
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the commutation interrupt (COMIE) is enabled.
  * @rmtoll DIER         COMIE         LL_TIM_IsEnabledIT_COM
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable trigger interrupt (TIE).
  * @rmtoll DIER         TIE           LL_TIM_EnableIT_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable trigger interrupt (TIE).
  * @rmtoll DIER         TIE           LL_TIM_DisableIT_TRIG
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the trigger interrupt (TIE) is enabled.
  * @rmtoll DIER         TIE           LL_TIM_IsEnabledIT_TRIG
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable break interrupt (BIE).
  * @rmtoll DIER         BIE           LL_TIM_EnableIT_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable break interrupt (BIE).
  * @rmtoll DIER         BIE           LL_TIM_DisableIT_BRK
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the break interrupt (BIE) is enabled.
  * @rmtoll DIER         BIE           LL_TIM_IsEnabledIT_BRK
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup TIM_LL_EF_DMA_Management DMA-Management
  * @{
  */
/* *
  * @brief  Enable update DMA request (UDE).
  * @rmtoll DIER         UDE           LL_TIM_EnableDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable update DMA request (UDE).
  * @rmtoll DIER         UDE           LL_TIM_DisableDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the update DMA request  (UDE) is enabled.
  * @rmtoll DIER         UDE           LL_TIM_IsEnabledDMAReq_UPDATE
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 1 DMA request (CC1DE).
  * @rmtoll DIER         CC1DE         LL_TIM_EnableDMAReq_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 1  DMA request (CC1DE).
  * @rmtoll DIER         CC1DE         LL_TIM_DisableDMAReq_CC1
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 1 DMA request (CC1DE) is enabled.
  * @rmtoll DIER         CC1DE         LL_TIM_IsEnabledDMAReq_CC1
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 2 DMA request (CC2DE).
  * @rmtoll DIER         CC2DE         LL_TIM_EnableDMAReq_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 2  DMA request (CC2DE).
  * @rmtoll DIER         CC2DE         LL_TIM_DisableDMAReq_CC2
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 2 DMA request (CC2DE) is enabled.
  * @rmtoll DIER         CC2DE         LL_TIM_IsEnabledDMAReq_CC2
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 3 DMA request (CC3DE).
  * @rmtoll DIER         CC3DE         LL_TIM_EnableDMAReq_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 3  DMA request (CC3DE).
  * @rmtoll DIER         CC3DE         LL_TIM_DisableDMAReq_CC3
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 3 DMA request (CC3DE) is enabled.
  * @rmtoll DIER         CC3DE         LL_TIM_IsEnabledDMAReq_CC3
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable capture/compare 4 DMA request (CC4DE).
  * @rmtoll DIER         CC4DE         LL_TIM_EnableDMAReq_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Disable capture/compare 4  DMA request (CC4DE).
  * @rmtoll DIER         CC4DE         LL_TIM_DisableDMAReq_CC4
  * @param  TIMx Timer instance
  * @retval None
  */
/* *
  * @brief  Indicates whether the capture/compare 4 DMA request (CC4DE) is enabled.
  * @rmtoll DIER         CC4DE         LL_TIM_IsEnabledDMAReq_CC4
  * @param  TIMx Timer instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable commutation DMA request (COMDE).
  * @rmtoll DIER         COMDE         LL_TIM_EnableDMAReq_COM
  * @param  TIMx Timer instance
  * @retval None
  */
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
