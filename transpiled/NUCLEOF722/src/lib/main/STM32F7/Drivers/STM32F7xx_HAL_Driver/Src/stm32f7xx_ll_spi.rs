use ::libc;
extern "C" {
    /* DFSDM1_Channel0 */
    #[no_mangle]
    fn LL_RCC_GetI2SClockFreq(I2SxSource: uint32_t) -> uint32_t;
}
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
pub struct SPI_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SR: uint32_t,
    pub DR: uint32_t,
    pub CRCPR: uint32_t,
    pub RXCRCR: uint32_t,
    pub TXCRCR: uint32_t,
    pub I2SCFGR: uint32_t,
    pub I2SPR: uint32_t,
}
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_SPI_InitTypeDef {
    pub TransferDirection: uint32_t,
    pub Mode: uint32_t,
    pub DataWidth: uint32_t,
    pub ClockPolarity: uint32_t,
    pub ClockPhase: uint32_t,
    pub NSS: uint32_t,
    pub BaudRate: uint32_t,
    pub BitOrder: uint32_t,
    pub CRCCalculation: uint32_t,
    pub CRCPoly: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_I2S_InitTypeDef {
    pub Mode: uint32_t,
    pub Standard: uint32_t,
    pub DataFormat: uint32_t,
    pub MCLKOutput: uint32_t,
    pub AudioFreq: uint32_t,
    pub ClockPolarity: uint32_t,
}
#[inline]
unsafe extern "C" fn LL_SPI_IsEnabled(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return ((*SPIx).CR1 & (0x1 as libc::c_uint) << 6 as libc::c_uint ==
                (0x1 as libc::c_uint) << 6 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_SPI_SetCRCPolynomial(mut SPIx: *mut SPI_TypeDef,
                                             mut CRCPoly: uint32_t) {
    ::core::ptr::write_volatile(&mut (*SPIx).CRCPR as *mut uint32_t,
                                CRCPoly as uint16_t as uint32_t);
}
#[inline]
unsafe extern "C" fn LL_I2S_IsEnabled(mut SPIx: *mut SPI_TypeDef)
 -> uint32_t {
    return ((*SPIx).I2SCFGR & (0x1 as libc::c_uint) << 10 as libc::c_uint ==
                (0x1 as libc::c_uint) << 10 as libc::c_uint) as libc::c_int as
               uint32_t;
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
  ******************************************************************************
  * @file    stm32f7xx_ll_spi.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   SPI LL module driver.
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
/* * @addtogroup SPI_LL
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* * @defgroup SPI_LL_Private_Constants SPI Private Constants
  * @{
  */
/* SPI registers Masks */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup SPI_LL_Private_Macros SPI Private Macros
  * @{
  */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup SPI_LL_Exported_Functions
  * @{
  */
/* * @addtogroup SPI_LL_EF_Init
  * @{
  */
/* *
  * @brief  De-initialize the SPI registers to their default reset values.
  * @param  SPIx SPI Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are de-initialized
  *          - ERROR: SPI registers are not de-initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_SPI_DeInit(mut SPIx: *mut SPI_TypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    if SPIx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3000
                                                                              as
                                                                              libc::c_uint)
               as *mut SPI_TypeDef {
        /* Force reset of SPI clock */
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 12 as libc::c_uint);
        /* Release reset of SPI clock */
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      12 as libc::c_uint);
        status = SUCCESS
    }
    /* SPI1 */
    if SPIx ==
           (0x40000000 as libc::c_uint).wrapping_add(0x3800 as libc::c_uint)
               as *mut SPI_TypeDef {
        /* Force reset of SPI clock */
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 14 as libc::c_uint);
        /* Release reset of SPI clock */
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      14 as libc::c_uint);
        status = SUCCESS
    }
    /* SPI2 */
    if SPIx ==
           (0x40000000 as libc::c_uint).wrapping_add(0x3c00 as libc::c_uint)
               as *mut SPI_TypeDef {
        /* Force reset of SPI clock */
        LL_APB1_GRP1_ForceReset((0x1 as libc::c_uint) << 15 as libc::c_uint);
        /* Release reset of SPI clock */
        LL_APB1_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      15 as libc::c_uint);
        status = SUCCESS
    }
    /* SPI3 */
    if SPIx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x3400
                                                                              as
                                                                              libc::c_uint)
               as *mut SPI_TypeDef {
        /* Force reset of SPI clock */
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 13 as libc::c_uint);
        /* Release reset of SPI clock */
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      13 as libc::c_uint);
        status = SUCCESS
    }
    /* SPI4 */
    if SPIx ==
           (0x40000000 as
                libc::c_uint).wrapping_add(0x10000 as
                                               libc::c_uint).wrapping_add(0x5000
                                                                              as
                                                                              libc::c_uint)
               as *mut SPI_TypeDef {
        /* Force reset of SPI clock */
        LL_APB2_GRP1_ForceReset((0x1 as libc::c_uint) << 20 as libc::c_uint);
        /* Release reset of SPI clock */
        LL_APB2_GRP1_ReleaseReset((0x1 as libc::c_uint) <<
                                      20 as libc::c_uint);
        status = SUCCESS
    }
    /* SPI5 */
    /* SPI6 */
    return status;
}
/* *
  * @brief  Initialize the SPI registers according to the specified parameters in SPI_InitStruct.
  * @note   As some bits in SPI configuration registers can only be written when the SPI is disabled (SPI_CR1_SPE bit =0),
  *         SPI IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  SPIx SPI Instance
  * @param  SPI_InitStruct pointer to a @ref LL_SPI_InitTypeDef structure
  * @retval An ErrorStatus enumeration value. (Return always SUCCESS)
  */
#[no_mangle]
pub unsafe extern "C" fn LL_SPI_Init(mut SPIx: *mut SPI_TypeDef,
                                     mut SPI_InitStruct:
                                         *mut LL_SPI_InitTypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    /* Check the SPI Instance SPIx*/
    if LL_SPI_IsEnabled(SPIx) == 0 as libc::c_uint {
        /*---------------------------- SPIx CR1 Configuration ------------------------
     * Configure SPIx CR1 with parameters:
     * - TransferDirection:  SPI_CR1_BIDIMODE, SPI_CR1_BIDIOE and SPI_CR1_RXONLY bits
     * - Master/Slave Mode:  SPI_CR1_MSTR bit
     * - ClockPolarity:      SPI_CR1_CPOL bit
     * - ClockPhase:         SPI_CR1_CPHA bit
     * - NSS management:     SPI_CR1_SSM bit
     * - BaudRate prescaler: SPI_CR1_BR[2:0] bits
     * - BitOrder:           SPI_CR1_LSBFIRST bit
     * - CRCCalculation:     SPI_CR1_CRCEN bit
     */
        ::core::ptr::write_volatile(&mut (*SPIx).CR1 as *mut uint32_t,
                                    (*SPIx).CR1 &
                                        !((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint |
                                              (0x7 as libc::c_uint) <<
                                                  3 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  7 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  9 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  10 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  11 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  12 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  13 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  14 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  15 as libc::c_uint) |
                                        ((*SPI_InitStruct).TransferDirection |
                                             (*SPI_InitStruct).Mode |
                                             (*SPI_InitStruct).ClockPolarity |
                                             (*SPI_InitStruct).ClockPhase |
                                             (*SPI_InitStruct).NSS |
                                             (*SPI_InitStruct).BaudRate |
                                             (*SPI_InitStruct).BitOrder |
                                             (*SPI_InitStruct).CRCCalculation));
        /*---------------------------- SPIx CR2 Configuration ------------------------
     * Configure SPIx CR2 with parameters:
     * - DataWidth:          DS[3:0] bits
     * - NSS management:     SSOE bit
     */
        ::core::ptr::write_volatile(&mut (*SPIx).CR2 as *mut uint32_t,
                                    (*SPIx).CR2 &
                                        !((0xf as libc::c_uint) <<
                                              8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint) |
                                        ((*SPI_InitStruct).DataWidth |
                                             (*SPI_InitStruct).NSS >>
                                                 16 as libc::c_uint));
        /*---------------------------- SPIx CRCPR Configuration ----------------------
     * Configure SPIx CRCPR with parameters:
     * - CRCPoly:            CRCPOLY[15:0] bits
     */
        if (*SPI_InitStruct).CRCCalculation ==
               (0x1 as libc::c_uint) << 13 as libc::c_uint {
            LL_SPI_SetCRCPolynomial(SPIx, (*SPI_InitStruct).CRCPoly);
        }
        status = SUCCESS
    }
    /* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*SPIx).I2SCFGR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           11 as libc::c_uint)) as uint32_t as
                                    uint32_t);
    return status;
}
/* *
  * @brief  Set each @ref LL_SPI_InitTypeDef field to default value.
  * @param  SPI_InitStruct pointer to a @ref LL_SPI_InitTypeDef structure
  * whose fields will be set to default values.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_SPI_StructInit(mut SPI_InitStruct:
                                               *mut LL_SPI_InitTypeDef) {
    /* Set SPI_InitStruct fields to default values */
    (*SPI_InitStruct).TransferDirection = 0 as libc::c_uint;
    (*SPI_InitStruct).Mode = 0 as libc::c_uint;
    (*SPI_InitStruct).DataWidth =
        (0x4 as libc::c_uint) << 8 as libc::c_uint |
            (0x2 as libc::c_uint) << 8 as libc::c_uint |
            (0x1 as libc::c_uint) << 8 as libc::c_uint;
    (*SPI_InitStruct).ClockPolarity = 0 as libc::c_uint;
    (*SPI_InitStruct).ClockPhase = 0 as libc::c_uint;
    (*SPI_InitStruct).NSS = 0 as libc::c_uint;
    (*SPI_InitStruct).BaudRate = 0 as libc::c_uint;
    (*SPI_InitStruct).BitOrder = 0 as libc::c_uint;
    (*SPI_InitStruct).CRCCalculation = 0 as libc::c_uint;
    (*SPI_InitStruct).CRCPoly = 7 as libc::c_uint;
}
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup I2S_LL
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* * @defgroup I2S_LL_Private_Constants I2S Private Constants
  * @{
  */
/* I2S registers Masks */
/* *
  * @}
  */
/* Private macros ------------------------------------------------------------*/
/* * @defgroup I2S_LL_Private_Macros I2S Private Macros
  * @{
  */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup I2S_LL_Exported_Functions
  * @{
  */
/* * @addtogroup I2S_LL_EF_Init
  * @{
  */
/* *
  * @brief  De-initialize the SPI/I2S registers to their default reset values.
  * @param  SPIx SPI Instance
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are de-initialized
  *          - ERROR: SPI registers are not de-initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_I2S_DeInit(mut SPIx: *mut SPI_TypeDef)
 -> ErrorStatus {
    return LL_SPI_DeInit(SPIx);
}
/* *
  * @brief  Initializes the SPI/I2S registers according to the specified parameters in I2S_InitStruct.
  * @note   As some bits in SPI configuration registers can only be written when the SPI is disabled (SPI_CR1_SPE bit =0),
  *         SPI IP should be in disabled state prior calling this function. Otherwise, ERROR result will be returned.
  * @param  SPIx SPI Instance
  * @param  I2S_InitStruct pointer to a @ref LL_I2S_InitTypeDef structure
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: SPI registers are Initialized
  *          - ERROR: SPI registers are not Initialized
  */
#[no_mangle]
pub unsafe extern "C" fn LL_I2S_Init(mut SPIx: *mut SPI_TypeDef,
                                     mut I2S_InitStruct:
                                         *mut LL_I2S_InitTypeDef)
 -> ErrorStatus {
    let mut i2sdiv: uint16_t = 2 as libc::c_uint as uint16_t;
    let mut i2sodd: uint16_t = 0 as libc::c_uint as uint16_t;
    let mut packetlength: uint16_t = 1 as libc::c_uint as uint16_t;
    let mut tmp: uint32_t = 0 as libc::c_uint;
    let mut sourceclock: uint32_t = 0 as libc::c_uint;
    let mut status: ErrorStatus = ERROR;
    /* Check the I2S parameters */
    if LL_I2S_IsEnabled(SPIx) == 0 as libc::c_uint {
        /*---------------------------- SPIx I2SCFGR Configuration --------------------
     * Configure SPIx I2SCFGR with parameters:
     * - Mode:          SPI_I2SCFGR_I2SCFG[1:0] bit
     * - Standard:      SPI_I2SCFGR_I2SSTD[1:0] and SPI_I2SCFGR_PCMSYNC bits
     * - DataFormat:    SPI_I2SCFGR_CHLEN and SPI_I2SCFGR_DATLEN bits
     * - ClockPolarity: SPI_I2SCFGR_CKPOL bit
     */
        /* Write to SPIx I2SCFGR */
        ::core::ptr::write_volatile(&mut (*SPIx).I2SCFGR as *mut uint32_t,
                                    (*SPIx).I2SCFGR &
                                        !((0x1 as libc::c_uint) <<
                                              0 as libc::c_uint |
                                              (0x3 as libc::c_uint) <<
                                                  1 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint |
                                              (0x3 as libc::c_uint) <<
                                                  4 as libc::c_uint |
                                              (0x3 as libc::c_uint) <<
                                                  8 as libc::c_uint |
                                              (0x1 as libc::c_uint) <<
                                                  11 as libc::c_uint) |
                                        ((*I2S_InitStruct).Mode |
                                             (*I2S_InitStruct).Standard |
                                             (*I2S_InitStruct).DataFormat |
                                             (*I2S_InitStruct).ClockPolarity |
                                             (0x1 as libc::c_uint) <<
                                                 11 as libc::c_uint));
        /*---------------------------- SPIx I2SPR Configuration ----------------------
     * Configure SPIx I2SPR with parameters:
     * - MCLKOutput:    SPI_I2SPR_MCKOE bit
     * - AudioFreq:     SPI_I2SPR_I2SDIV[7:0] and SPI_I2SPR_ODD bits
     */
        /* If the requested audio frequency is not the default, compute the prescaler (i2sodd, i2sdiv)
     * else, default values are used:  i2sodd = 0U, i2sdiv = 2U.
     */
        if (*I2S_InitStruct).AudioFreq != 2 as libc::c_uint {
            /* Check the frame length (For the Prescaler computing)
       * Default value: LL_I2S_DATAFORMAT_16B (packetlength = 1U).
       */
            if (*I2S_InitStruct).DataFormat != 0 as libc::c_uint {
                /* Packet length is 32 bits */
                packetlength = 2 as libc::c_uint as uint16_t
            }
            /* If an external I2S clock has to be used, the specific define should be set
      in the project configuration or in the stm32f7xx_ll_rcc.h file */
      /* Get the I2S source clock value */
            sourceclock =
                LL_RCC_GetI2SClockFreq((0x1 as libc::c_uint) <<
                                           23 as libc::c_uint);
            /* Compute the Real divider depending on the MCLK output state with a floating point */
            if (*I2S_InitStruct).MCLKOutput ==
                   (0x1 as libc::c_uint) << 9 as libc::c_uint {
                /* MCLK output is enabled */
                tmp =
                    sourceclock.wrapping_div(256 as
                                                 libc::c_uint).wrapping_mul(10
                                                                                as
                                                                                libc::c_uint).wrapping_div((*I2S_InitStruct).AudioFreq).wrapping_add(5
                                                                                                                                                         as
                                                                                                                                                         libc::c_uint)
                        as uint16_t as uint32_t
            } else {
                /* MCLK output is disabled */
                tmp =
                    sourceclock.wrapping_div((32 as
                                                  libc::c_uint).wrapping_mul(packetlength
                                                                                 as
                                                                                 libc::c_uint)).wrapping_mul(10
                                                                                                                 as
                                                                                                                 libc::c_uint).wrapping_div((*I2S_InitStruct).AudioFreq).wrapping_add(5
                                                                                                                                                                                          as
                                                                                                                                                                                          libc::c_uint)
                        as uint16_t as uint32_t
            }
            /* Remove the floating point */
            tmp = tmp.wrapping_div(10 as libc::c_uint);
            /* Check the parity of the divider */
            i2sodd =
                (tmp & 0x1 as libc::c_uint as uint16_t as libc::c_uint) as
                    uint16_t;
            /* Compute the i2sdiv prescaler */
            i2sdiv =
                tmp.wrapping_sub(i2sodd as
                                     libc::c_uint).wrapping_div(2 as
                                                                    libc::c_uint)
                    as uint16_t;
            /* Get the Mask for the Odd bit (SPI_I2SPR[8]) register */
            i2sodd =
                ((i2sodd as libc::c_int) << 8 as libc::c_uint) as uint16_t
        }
        /* Test if the divider is 1 or 0 or greater than 0xFF */
        if (i2sdiv as libc::c_uint) < 2 as libc::c_uint ||
               i2sdiv as libc::c_uint > 0xff as libc::c_uint {
            /* Set the default values */
            i2sdiv = 2 as libc::c_uint as uint16_t;
            i2sodd = 0 as libc::c_uint as uint16_t
        }
        /* Write to SPIx I2SPR register the computed value */
        ::core::ptr::write_volatile(&mut (*SPIx).I2SPR as *mut uint32_t,
                                    (i2sdiv as libc::c_int |
                                         i2sodd as libc::c_int) as
                                        libc::c_uint |
                                        (*I2S_InitStruct).MCLKOutput);
        status = SUCCESS
    }
    return status;
}
/* *
  * @brief  Set each @ref LL_I2S_InitTypeDef field to default value.
  * @param  I2S_InitStruct pointer to a @ref LL_I2S_InitTypeDef structure
  *         whose fields will be set to default values.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_I2S_StructInit(mut I2S_InitStruct:
                                               *mut LL_I2S_InitTypeDef) {
    /*--------------- Reset I2S init structure parameters values -----------------*/
    (*I2S_InitStruct).Mode = 0 as libc::c_uint;
    (*I2S_InitStruct).Standard = 0 as libc::c_uint;
    (*I2S_InitStruct).DataFormat = 0 as libc::c_uint;
    (*I2S_InitStruct).MCLKOutput = 0 as libc::c_uint;
    (*I2S_InitStruct).AudioFreq = 2 as libc::c_uint;
    (*I2S_InitStruct).ClockPolarity = 0 as libc::c_uint;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_spi.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of SPI LL module.
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
/* * @defgroup SPI_LL SPI
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup SPI_LL_ES_INIT SPI Exported Init structure
  * @{
  */
/* *
  * @brief  SPI Init structures definition
  */
/* !< Specifies the SPI unidirectional or bidirectional data mode.
                                         This parameter can be a value of @ref SPI_LL_EC_TRANSFER_MODE.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetTransferDirection().*/
/* !< Specifies the SPI mode (Master/Slave).
                                         This parameter can be a value of @ref SPI_LL_EC_MODE.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetMode().*/
/* !< Specifies the SPI data width.
                                         This parameter can be a value of @ref SPI_LL_EC_DATAWIDTH.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetDataWidth().*/
/* !< Specifies the serial clock steady state.
                                         This parameter can be a value of @ref SPI_LL_EC_POLARITY.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetClockPolarity().*/
/* !< Specifies the clock active edge for the bit capture.
                                         This parameter can be a value of @ref SPI_LL_EC_PHASE.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetClockPhase().*/
/* !< Specifies whether the NSS signal is managed by hardware (NSS pin) or by software using the SSI bit.
                                         This parameter can be a value of @ref SPI_LL_EC_NSS_MODE.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetNSSMode().*/
/* !< Specifies the BaudRate prescaler value which will be used to configure the transmit and receive SCK clock.
                                         This parameter can be a value of @ref SPI_LL_EC_BAUDRATEPRESCALER.
                                         @note The communication clock is derived from the master clock. The slave clock does not need to be set.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetBaudRatePrescaler().*/
/* !< Specifies whether data transfers start from MSB or LSB bit.
                                         This parameter can be a value of @ref SPI_LL_EC_BIT_ORDER.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetTransferBitOrder().*/
/* !< Specifies if the CRC calculation is enabled or not.
                                         This parameter can be a value of @ref SPI_LL_EC_CRC_CALCULATION.

                                         This feature can be modified afterwards using unitary functions @ref LL_SPI_EnableCRC() and @ref LL_SPI_DisableCRC().*/
/* !< Specifies the polynomial used for the CRC calculation.
                                         This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFFFF.

                                         This feature can be modified afterwards using unitary function @ref LL_SPI_SetCRCPolynomial().*/
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup SPI_LL_Exported_Constants SPI Exported Constants
  * @{
  */
/* * @defgroup SPI_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_SPI_ReadReg function
  * @{
  */
/* !< Rx buffer not empty flag         */
/* !< Tx buffer empty flag             */
/* !< Busy flag                        */
/* !< CRC error flag                   */
/* !< Mode fault flag                  */
/* !< Overrun flag                     */
/* !< TI mode frame format error flag  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_SPI_ReadReg and  LL_SPI_WriteReg functions
  * @{
  */
/* !< Rx buffer not empty interrupt enable */
/* !< Tx buffer empty interrupt enable     */
/* !< Error interrupt enable               */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_MODE Operation Mode
  * @{
  */
/* !< Master configuration  */
/* !< Slave configuration   */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_PROTOCOL Serial Protocol
  * @{
  */
/* !< Motorola mode. Used as default value */
/* !< TI mode                              */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_PHASE Clock Phase
  * @{
  */
/* !< First clock transition is the first data capture edge  */
/* !< Second clock transition is the first data capture edge */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_POLARITY Clock Polarity
  * @{
  */
/* !< Clock to 0 when idle */
/* !< Clock to 1 when idle */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_BAUDRATEPRESCALER Baud Rate Prescaler
  * @{
  */
/* !< BaudRate control equal to fPCLK/2   */
/* !< BaudRate control equal to fPCLK/4   */
/* !< BaudRate control equal to fPCLK/8   */
/* !< BaudRate control equal to fPCLK/16  */
/* !< BaudRate control equal to fPCLK/32  */
/* !< BaudRate control equal to fPCLK/64  */
/* !< BaudRate control equal to fPCLK/128 */
/* !< BaudRate control equal to fPCLK/256 */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_BIT_ORDER Transmission Bit Order
  * @{
  */
/* !< Data is transmitted/received with the LSB first */
/* !< Data is transmitted/received with the MSB first */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_TRANSFER_MODE Transfer Mode
  * @{
  */
/* !< Full-Duplex mode. Rx and Tx transfer on 2 lines */
/* !< Simplex Rx mode.  Rx transfer only on 1 line    */
/* !< Half-Duplex Rx mode. Rx transfer on 1 line      */
/* !< Half-Duplex Tx mode. Tx transfer on 1 line      */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_NSS_MODE Slave Select Pin Mode
  * @{
  */
/* !< NSS managed internally. NSS pin not used and free              */
/* !< NSS pin used in Input. Only used in Master mode                */
/* !< NSS pin used in Output. Only used in Slave mode as chip select */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_DATAWIDTH Datawidth
  * @{
  */
/* !< Data length for SPI transfer:  4 bits */
/* !< Data length for SPI transfer:  5 bits */
/* !< Data length for SPI transfer:  6 bits */
/* !< Data length for SPI transfer:  7 bits */
/* !< Data length for SPI transfer:  8 bits */
/* !< Data length for SPI transfer:  9 bits */
/* !< Data length for SPI transfer: 10 bits */
/* !< Data length for SPI transfer: 11 bits */
/* !< Data length for SPI transfer: 12 bits */
/* !< Data length for SPI transfer: 13 bits */
/* !< Data length for SPI transfer: 14 bits */
/* !< Data length for SPI transfer: 15 bits */
/* !< Data length for SPI transfer: 16 bits */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_CRC_CALCULATION CRC Calculation
  * @{
  */
/* !< CRC calculation disabled */
/* !< CRC calculation enabled  */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* * @defgroup SPI_LL_EC_CRC_LENGTH CRC Length
  * @{
  */
/* !<  8-bit CRC length */
/* !< 16-bit CRC length */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_RX_FIFO_TH RX FIFO Threshold
  * @{
  */
/* !< RXNE event is generated if FIFO level is greater than or equel to 1/2 (16-bit) */
/* !< RXNE event is generated if FIFO level is greater than or equel to 1/4 (8-bit)  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_RX_FIFO RX FIFO Level
  * @{
  */
/* !< FIFO reception empty */
/* !< FIFO reception 1/4   */
/* !< FIFO reception 1/2   */
/* !< FIFO reception full  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_TX_FIFO TX FIFO Level
  * @{
  */
/* !< FIFO transmission empty */
/* !< FIFO transmission 1/4   */
/* !< FIFO transmission 1/2   */
/* !< FIFO transmission full  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_DMA_PARITY DMA Parity
  * @{
  */
/* !< Select DMA parity Even */
/* !< Select DMA parity Odd  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup SPI_LL_Exported_Macros SPI Exported Macros
  * @{
  */
/* * @defgroup SPI_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in SPI register
  * @param  __INSTANCE__ SPI Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in SPI register
  * @param  __INSTANCE__ SPI Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup SPI_LL_Exported_Functions SPI Exported Functions
  * @{
  */
/* * @defgroup SPI_LL_EF_Configuration Configuration
  * @{
  */
/* *
  * @brief  Enable SPI peripheral
  * @rmtoll CR1          SPE           LL_SPI_Enable
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable SPI peripheral
  * @note   When disabling the SPI, follow the procedure described in the Reference Manual.
  * @rmtoll CR1          SPE           LL_SPI_Disable
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if SPI peripheral is enabled
  * @rmtoll CR1          SPE           LL_SPI_IsEnabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set SPI operation mode to Master or Slave
  * @note   This bit should not be changed when communication is ongoing.
  * @rmtoll CR1          MSTR          LL_SPI_SetMode\n
  *         CR1          SSI           LL_SPI_SetMode
  * @param  SPIx SPI Instance
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_SPI_MODE_MASTER
  *         @arg @ref LL_SPI_MODE_SLAVE
  * @retval None
  */
/* *
  * @brief  Get SPI operation mode (Master or Slave)
  * @rmtoll CR1          MSTR          LL_SPI_GetMode\n
  *         CR1          SSI           LL_SPI_GetMode
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_MODE_MASTER
  *         @arg @ref LL_SPI_MODE_SLAVE
  */
/* *
  * @brief  Set serial protocol used
  * @note   This bit should be written only when SPI is disabled (SPE = 0) for correct operation.
  * @rmtoll CR2          FRF           LL_SPI_SetStandard
  * @param  SPIx SPI Instance
  * @param  Standard This parameter can be one of the following values:
  *         @arg @ref LL_SPI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_SPI_PROTOCOL_TI
  * @retval None
  */
/* *
  * @brief  Get serial protocol used
  * @rmtoll CR2          FRF           LL_SPI_GetStandard
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_PROTOCOL_MOTOROLA
  *         @arg @ref LL_SPI_PROTOCOL_TI
  */
/* *
  * @brief  Set clock phase
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  * @rmtoll CR1          CPHA          LL_SPI_SetClockPhase
  * @param  SPIx SPI Instance
  * @param  ClockPhase This parameter can be one of the following values:
  *         @arg @ref LL_SPI_PHASE_1EDGE
  *         @arg @ref LL_SPI_PHASE_2EDGE
  * @retval None
  */
/* *
  * @brief  Get clock phase
  * @rmtoll CR1          CPHA          LL_SPI_GetClockPhase
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_PHASE_1EDGE
  *         @arg @ref LL_SPI_PHASE_2EDGE
  */
/* *
  * @brief  Set clock polarity
  * @note   This bit should not be changed when communication is ongoing.
  *         This bit is not used in SPI TI mode.
  * @rmtoll CR1          CPOL          LL_SPI_SetClockPolarity
  * @param  SPIx SPI Instance
  * @param  ClockPolarity This parameter can be one of the following values:
  *         @arg @ref LL_SPI_POLARITY_LOW
  *         @arg @ref LL_SPI_POLARITY_HIGH
  * @retval None
  */
/* *
  * @brief  Get clock polarity
  * @rmtoll CR1          CPOL          LL_SPI_GetClockPolarity
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_POLARITY_LOW
  *         @arg @ref LL_SPI_POLARITY_HIGH
  */
/* *
  * @brief  Set baud rate prescaler
  * @note   These bits should not be changed when communication is ongoing. SPI BaudRate = fPCLK/Prescaler.
  * @rmtoll CR1          BR            LL_SPI_SetBaudRatePrescaler
  * @param  SPIx SPI Instance
  * @param  BaudRate This parameter can be one of the following values:
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV2
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV4
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV8
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV16
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV32
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV64
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV128
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV256
  * @retval None
  */
/* *
  * @brief  Get baud rate prescaler
  * @rmtoll CR1          BR            LL_SPI_GetBaudRatePrescaler
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV2
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV4
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV8
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV16
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV32
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV64
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV128
  *         @arg @ref LL_SPI_BAUDRATEPRESCALER_DIV256
  */
/* *
  * @brief  Set transfer bit order
  * @note   This bit should not be changed when communication is ongoing. This bit is not used in SPI TI mode.
  * @rmtoll CR1          LSBFIRST      LL_SPI_SetTransferBitOrder
  * @param  SPIx SPI Instance
  * @param  BitOrder This parameter can be one of the following values:
  *         @arg @ref LL_SPI_LSB_FIRST
  *         @arg @ref LL_SPI_MSB_FIRST
  * @retval None
  */
/* *
  * @brief  Get transfer bit order
  * @rmtoll CR1          LSBFIRST      LL_SPI_GetTransferBitOrder
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_LSB_FIRST
  *         @arg @ref LL_SPI_MSB_FIRST
  */
/* *
  * @brief  Set transfer direction mode
  * @note   For Half-Duplex mode, Rx Direction is set by default.
  *         In master mode, the MOSI pin is used and in slave mode, the MISO pin is used for Half-Duplex.
  * @rmtoll CR1          RXONLY        LL_SPI_SetTransferDirection\n
  *         CR1          BIDIMODE      LL_SPI_SetTransferDirection\n
  *         CR1          BIDIOE        LL_SPI_SetTransferDirection
  * @param  SPIx SPI Instance
  * @param  TransferDirection This parameter can be one of the following values:
  *         @arg @ref LL_SPI_FULL_DUPLEX
  *         @arg @ref LL_SPI_SIMPLEX_RX
  *         @arg @ref LL_SPI_HALF_DUPLEX_RX
  *         @arg @ref LL_SPI_HALF_DUPLEX_TX
  * @retval None
  */
/* *
  * @brief  Get transfer direction mode
  * @rmtoll CR1          RXONLY        LL_SPI_GetTransferDirection\n
  *         CR1          BIDIMODE      LL_SPI_GetTransferDirection\n
  *         CR1          BIDIOE        LL_SPI_GetTransferDirection
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_FULL_DUPLEX
  *         @arg @ref LL_SPI_SIMPLEX_RX
  *         @arg @ref LL_SPI_HALF_DUPLEX_RX
  *         @arg @ref LL_SPI_HALF_DUPLEX_TX
  */
/* *
  * @brief  Set frame data width
  * @rmtoll CR2          DS            LL_SPI_SetDataWidth
  * @param  SPIx SPI Instance
  * @param  DataWidth This parameter can be one of the following values:
  *         @arg @ref LL_SPI_DATAWIDTH_4BIT
  *         @arg @ref LL_SPI_DATAWIDTH_5BIT
  *         @arg @ref LL_SPI_DATAWIDTH_6BIT
  *         @arg @ref LL_SPI_DATAWIDTH_7BIT
  *         @arg @ref LL_SPI_DATAWIDTH_8BIT
  *         @arg @ref LL_SPI_DATAWIDTH_9BIT
  *         @arg @ref LL_SPI_DATAWIDTH_10BIT
  *         @arg @ref LL_SPI_DATAWIDTH_11BIT
  *         @arg @ref LL_SPI_DATAWIDTH_12BIT
  *         @arg @ref LL_SPI_DATAWIDTH_13BIT
  *         @arg @ref LL_SPI_DATAWIDTH_14BIT
  *         @arg @ref LL_SPI_DATAWIDTH_15BIT
  *         @arg @ref LL_SPI_DATAWIDTH_16BIT
  * @retval None
  */
/* *
  * @brief  Get frame data width
  * @rmtoll CR2          DS            LL_SPI_GetDataWidth
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_DATAWIDTH_4BIT
  *         @arg @ref LL_SPI_DATAWIDTH_5BIT
  *         @arg @ref LL_SPI_DATAWIDTH_6BIT
  *         @arg @ref LL_SPI_DATAWIDTH_7BIT
  *         @arg @ref LL_SPI_DATAWIDTH_8BIT
  *         @arg @ref LL_SPI_DATAWIDTH_9BIT
  *         @arg @ref LL_SPI_DATAWIDTH_10BIT
  *         @arg @ref LL_SPI_DATAWIDTH_11BIT
  *         @arg @ref LL_SPI_DATAWIDTH_12BIT
  *         @arg @ref LL_SPI_DATAWIDTH_13BIT
  *         @arg @ref LL_SPI_DATAWIDTH_14BIT
  *         @arg @ref LL_SPI_DATAWIDTH_15BIT
  *         @arg @ref LL_SPI_DATAWIDTH_16BIT
  */
/* *
  * @brief  Set threshold of RXFIFO that triggers an RXNE event
  * @rmtoll CR2          FRXTH         LL_SPI_SetRxFIFOThreshold
  * @param  SPIx SPI Instance
  * @param  Threshold This parameter can be one of the following values:
  *         @arg @ref LL_SPI_RX_FIFO_TH_HALF
  *         @arg @ref LL_SPI_RX_FIFO_TH_QUARTER
  * @retval None
  */
/* *
  * @brief  Get threshold of RXFIFO that triggers an RXNE event
  * @rmtoll CR2          FRXTH         LL_SPI_GetRxFIFOThreshold
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_RX_FIFO_TH_HALF
  *         @arg @ref LL_SPI_RX_FIFO_TH_QUARTER
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_CRC_Management CRC Management
  * @{
  */
/* *
  * @brief  Enable CRC
  * @note   This bit should be written only when SPI is disabled (SPE = 0) for correct operation.
  * @rmtoll CR1          CRCEN         LL_SPI_EnableCRC
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable CRC
  * @note   This bit should be written only when SPI is disabled (SPE = 0) for correct operation.
  * @rmtoll CR1          CRCEN         LL_SPI_DisableCRC
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if CRC is enabled
  * @note   This bit should be written only when SPI is disabled (SPE = 0) for correct operation.
  * @rmtoll CR1          CRCEN         LL_SPI_IsEnabledCRC
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set CRC Length
  * @note   This bit should be written only when SPI is disabled (SPE = 0) for correct operation.
  * @rmtoll CR1          CRCL          LL_SPI_SetCRCWidth
  * @param  SPIx SPI Instance
  * @param  CRCLength This parameter can be one of the following values:
  *         @arg @ref LL_SPI_CRC_8BIT
  *         @arg @ref LL_SPI_CRC_16BIT
  * @retval None
  */
/* *
  * @brief  Get CRC Length
  * @rmtoll CR1          CRCL          LL_SPI_GetCRCWidth
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_CRC_8BIT
  *         @arg @ref LL_SPI_CRC_16BIT
  */
/* *
  * @brief  Set CRCNext to transfer CRC on the line
  * @note   This bit has to be written as soon as the last data is written in the SPIx_DR register.
  * @rmtoll CR1          CRCNEXT       LL_SPI_SetCRCNext
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Set polynomial for CRC calculation
  * @rmtoll CRCPR        CRCPOLY       LL_SPI_SetCRCPolynomial
  * @param  SPIx SPI Instance
  * @param  CRCPoly This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFFFF
  * @retval None
  */
/* *
  * @brief  Get polynomial for CRC calculation
  * @rmtoll CRCPR        CRCPOLY       LL_SPI_GetCRCPolynomial
  * @param  SPIx SPI Instance
  * @retval Returned value is a number between Min_Data = 0x00 and Max_Data = 0xFFFF
  */
/* *
  * @brief  Get Rx CRC
  * @rmtoll RXCRCR       RXCRC         LL_SPI_GetRxCRC
  * @param  SPIx SPI Instance
  * @retval Returned value is a number between Min_Data = 0x00 and Max_Data = 0xFFFF
  */
/* *
  * @brief  Get Tx CRC
  * @rmtoll TXCRCR       TXCRC         LL_SPI_GetTxCRC
  * @param  SPIx SPI Instance
  * @retval Returned value is a number between Min_Data = 0x00 and Max_Data = 0xFFFF
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_NSS_Management Slave Select Pin Management
  * @{
  */
/* *
  * @brief  Set NSS mode
  * @note   LL_SPI_NSS_SOFT Mode is not used in SPI TI mode.
  * @rmtoll CR1          SSM           LL_SPI_SetNSSMode\n
  * @rmtoll CR2          SSOE          LL_SPI_SetNSSMode
  * @param  SPIx SPI Instance
  * @param  NSS This parameter can be one of the following values:
  *         @arg @ref LL_SPI_NSS_SOFT
  *         @arg @ref LL_SPI_NSS_HARD_INPUT
  *         @arg @ref LL_SPI_NSS_HARD_OUTPUT
  * @retval None
  */
/* *
  * @brief  Get NSS mode
  * @rmtoll CR1          SSM           LL_SPI_GetNSSMode\n
  * @rmtoll CR2          SSOE          LL_SPI_GetNSSMode
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_NSS_SOFT
  *         @arg @ref LL_SPI_NSS_HARD_INPUT
  *         @arg @ref LL_SPI_NSS_HARD_OUTPUT
  */
/* *
  * @brief  Enable NSS pulse management
  * @note   This bit should not be changed when communication is ongoing. This bit is not used in SPI TI mode.
  * @rmtoll CR2          NSSP          LL_SPI_EnableNSSPulseMgt
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable NSS pulse management
  * @note   This bit should not be changed when communication is ongoing. This bit is not used in SPI TI mode.
  * @rmtoll CR2          NSSP          LL_SPI_DisableNSSPulseMgt
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if NSS pulse is enabled
  * @note   This bit should not be changed when communication is ongoing. This bit is not used in SPI TI mode.
  * @rmtoll CR2          NSSP          LL_SPI_IsEnabledNSSPulse
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_FLAG_Management FLAG Management
  * @{
  */
/* *
  * @brief  Check if Rx buffer is not empty
  * @rmtoll SR           RXNE          LL_SPI_IsActiveFlag_RXNE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if Tx buffer is empty
  * @rmtoll SR           TXE           LL_SPI_IsActiveFlag_TXE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get CRC error flag
  * @rmtoll SR           CRCERR        LL_SPI_IsActiveFlag_CRCERR
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get mode fault error flag
  * @rmtoll SR           MODF          LL_SPI_IsActiveFlag_MODF
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get overrun error flag
  * @rmtoll SR           OVR           LL_SPI_IsActiveFlag_OVR
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get busy flag
  * @note   The BSY flag is cleared under any one of the following conditions:
  * -When the SPI is correctly disabled
  * -When a fault is detected in Master mode (MODF bit set to 1)
  * -In Master mode, when it finishes a data transmission and no new data is ready to be
  * sent
  * -In Slave mode, when the BSY flag is set to '0' for at least one SPI clock cycle between
  * each data transfer.
  * @rmtoll SR           BSY           LL_SPI_IsActiveFlag_BSY
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get frame format error flag
  * @rmtoll SR           FRE           LL_SPI_IsActiveFlag_FRE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get FIFO reception Level
  * @rmtoll SR           FRLVL         LL_SPI_GetRxFIFOLevel
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_RX_FIFO_EMPTY
  *         @arg @ref LL_SPI_RX_FIFO_QUARTER_FULL
  *         @arg @ref LL_SPI_RX_FIFO_HALF_FULL
  *         @arg @ref LL_SPI_RX_FIFO_FULL
  */
/* *
  * @brief  Get FIFO Transmission Level
  * @rmtoll SR           FTLVL         LL_SPI_GetTxFIFOLevel
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_TX_FIFO_EMPTY
  *         @arg @ref LL_SPI_TX_FIFO_QUARTER_FULL
  *         @arg @ref LL_SPI_TX_FIFO_HALF_FULL
  *         @arg @ref LL_SPI_TX_FIFO_FULL
  */
/* *
  * @brief  Clear CRC error flag
  * @rmtoll SR           CRCERR        LL_SPI_ClearFlag_CRCERR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Clear mode fault error flag
  * @note   Clearing this flag is done by a read access to the SPIx_SR
  *         register followed by a write access to the SPIx_CR1 register
  * @rmtoll SR           MODF          LL_SPI_ClearFlag_MODF
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Clear overrun error flag
  * @note   Clearing this flag is done by a read access to the SPIx_DR
  *         register followed by a read access to the SPIx_SR register
  * @rmtoll SR           OVR           LL_SPI_ClearFlag_OVR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Clear frame format error flag
  * @note   Clearing this flag is done by reading SPIx_SR register
  * @rmtoll SR           FRE           LL_SPI_ClearFlag_FRE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_IT_Management Interrupt Management
  * @{
  */
/* *
  * @brief  Enable error interrupt
  * @note   This bit controls the generation of an interrupt when an error condition occurs (CRCERR, OVR, MODF in SPI mode, FRE at TI mode).
  * @rmtoll CR2          ERRIE         LL_SPI_EnableIT_ERR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Enable Rx buffer not empty interrupt
  * @rmtoll CR2          RXNEIE        LL_SPI_EnableIT_RXNE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Enable Tx buffer empty interrupt
  * @rmtoll CR2          TXEIE         LL_SPI_EnableIT_TXE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable error interrupt
  * @note   This bit controls the generation of an interrupt when an error condition occurs (CRCERR, OVR, MODF in SPI mode, FRE at TI mode).
  * @rmtoll CR2          ERRIE         LL_SPI_DisableIT_ERR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable Rx buffer not empty interrupt
  * @rmtoll CR2          RXNEIE        LL_SPI_DisableIT_RXNE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable Tx buffer empty interrupt
  * @rmtoll CR2          TXEIE         LL_SPI_DisableIT_TXE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if error interrupt is enabled
  * @rmtoll CR2          ERRIE         LL_SPI_IsEnabledIT_ERR
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if Rx buffer not empty interrupt is enabled
  * @rmtoll CR2          RXNEIE        LL_SPI_IsEnabledIT_RXNE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if Tx buffer empty interrupt
  * @rmtoll CR2          TXEIE         LL_SPI_IsEnabledIT_TXE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_DMA_Management DMA Management
  * @{
  */
/* *
  * @brief  Enable DMA Rx
  * @rmtoll CR2          RXDMAEN       LL_SPI_EnableDMAReq_RX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable DMA Rx
  * @rmtoll CR2          RXDMAEN       LL_SPI_DisableDMAReq_RX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if DMA Rx is enabled
  * @rmtoll CR2          RXDMAEN       LL_SPI_IsEnabledDMAReq_RX
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable DMA Tx
  * @rmtoll CR2          TXDMAEN       LL_SPI_EnableDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable DMA Tx
  * @rmtoll CR2          TXDMAEN       LL_SPI_DisableDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if DMA Tx is enabled
  * @rmtoll CR2          TXDMAEN       LL_SPI_IsEnabledDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set parity of  Last DMA reception
  * @rmtoll CR2          LDMARX        LL_SPI_SetDMAParity_RX
  * @param  SPIx SPI Instance
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref LL_SPI_DMA_PARITY_ODD
  *         @arg @ref LL_SPI_DMA_PARITY_EVEN
  * @retval None
  */
/* *
  * @brief  Get parity configuration for  Last DMA reception
  * @rmtoll CR2          LDMARX        LL_SPI_GetDMAParity_RX
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_DMA_PARITY_ODD
  *         @arg @ref LL_SPI_DMA_PARITY_EVEN
  */
/* *
  * @brief  Set parity of  Last DMA transmission
  * @rmtoll CR2          LDMATX        LL_SPI_SetDMAParity_TX
  * @param  SPIx SPI Instance
  * @param  Parity This parameter can be one of the following values:
  *         @arg @ref LL_SPI_DMA_PARITY_ODD
  *         @arg @ref LL_SPI_DMA_PARITY_EVEN
  * @retval None
  */
/* *
  * @brief  Get parity configuration for Last DMA transmission
  * @rmtoll CR2          LDMATX        LL_SPI_GetDMAParity_TX
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_SPI_DMA_PARITY_ODD
  *         @arg @ref LL_SPI_DMA_PARITY_EVEN
  */
/* *
  * @brief  Get the data register address used for DMA transfer
  * @rmtoll DR           DR            LL_SPI_DMA_GetRegAddr
  * @param  SPIx SPI Instance
  * @retval Address of data register
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_DATA_Management DATA Management
  * @{
  */
/* *
  * @brief  Read 8-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_ReceiveData8
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x00 and Max_Data=0xFF
  */
/* *
  * @brief  Read 16-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_ReceiveData16
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x00 and Max_Data=0xFFFF
  */
/* *
  * @brief  Write 8-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_TransmitData8
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x00 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Write 16-Bits in the data register
  * @rmtoll DR           DR            LL_SPI_TransmitData16
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x00 and Max_Data=0xFFFF
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup I2S_LL I2S
  * @{
  */
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup I2S_LL_ES_INIT I2S Exported Init structure
  * @{
  */
/* *
  * @brief  I2S Init structure definition
  */
/* !< Specifies the I2S operating mode.
                                         This parameter can be a value of @ref I2S_LL_EC_MODE

                                         This feature can be modified afterwards using unitary function @ref LL_I2S_SetTransferMode().*/
/* !< Specifies the standard used for the I2S communication.
                                         This parameter can be a value of @ref I2S_LL_EC_STANDARD

                                         This feature can be modified afterwards using unitary function @ref LL_I2S_SetStandard().*/
/* !< Specifies the data format for the I2S communication.
                                         This parameter can be a value of @ref I2S_LL_EC_DATA_FORMAT

                                         This feature can be modified afterwards using unitary function @ref LL_I2S_SetDataFormat().*/
/* !< Specifies whether the I2S MCLK output is enabled or not.
                                         This parameter can be a value of @ref I2S_LL_EC_MCLK_OUTPUT

                                         This feature can be modified afterwards using unitary functions @ref LL_I2S_EnableMasterClock() or @ref LL_I2S_DisableMasterClock.*/
/* !< Specifies the frequency selected for the I2S communication.
                                         This parameter can be a value of @ref I2S_LL_EC_AUDIO_FREQ

                                         Audio Frequency can be modified afterwards using Reference manual formulas to calculate Prescaler Linear, Parity
                                         and unitary functions @ref LL_I2S_SetPrescalerLinear() and @ref LL_I2S_SetPrescalerParity() to set it.*/
/* !< Specifies the idle state of the I2S clock.
                                         This parameter can be a value of @ref I2S_LL_EC_POLARITY

                                         This feature can be modified afterwards using unitary function @ref LL_I2S_SetClockPolarity().*/
/* *
  * @}
  */
/*USE_FULL_LL_DRIVER*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup I2S_LL_Exported_Constants I2S Exported Constants
  * @{
  */
/* * @defgroup I2S_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_I2S_ReadReg function
  * @{
  */
/* !< Rx buffer not empty flag         */
/* !< Tx buffer empty flag             */
/* !< Busy flag                        */
/* !< Underrun flag                    */
/* !< Overrun flag                     */
/* !< TI mode frame format error flag  */
/* *
  * @}
  */
/* * @defgroup SPI_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_SPI_ReadReg and  LL_SPI_WriteReg functions
  * @{
  */
/* !< Rx buffer not empty interrupt enable */
/* !< Tx buffer empty interrupt enable     */
/* !< Error interrupt enable               */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_DATA_FORMAT Data format
  * @{
  */
/* !< Data length 16 bits, Channel lenght 16bit */
/* !< Data length 16 bits, Channel lenght 32bit */
/* !< Data length 24 bits, Channel lenght 32bit */
/* !< Data length 16 bits, Channel lenght 32bit */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_POLARITY Clock Polarity
  * @{
  */
/* !< Clock steady state is low level  */
/* !< Clock steady state is high level */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_STANDARD I2s Standard
  * @{
  */
/* !< I2S standard philips                      */
/* !< MSB justified standard (left justified)   */
/* !< LSB justified standard (right justified)  */
/* !< PCM standard, short frame synchronization */
/* !< PCM standard, long frame synchronization  */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_MODE Operation Mode
  * @{
  */
/* !< Slave Tx configuration  */
/* !< Slave Rx configuration  */
/* !< Master Tx configuration */
/* !< Master Rx configuration */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_PRESCALER_FACTOR Prescaler Factor
  * @{
  */
/* !< Odd factor: Real divider value is =  I2SDIV * 2    */
/* !< Odd factor: Real divider value is = (I2SDIV * 2)+1 */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_MCLK_OUTPUT MCLK Output
  * @{
  */
/* !< Master clock output is disabled */
/* !< Master clock output is enabled  */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EC_AUDIO_FREQ Audio Frequency
  * @{
  */
/* !< Audio Frequency configuration 192000 Hz       */
/* !< Audio Frequency configuration  96000 Hz       */
/* !< Audio Frequency configuration  48000 Hz       */
/* !< Audio Frequency configuration  44100 Hz       */
/* !< Audio Frequency configuration  32000 Hz       */
/* !< Audio Frequency configuration  22050 Hz       */
/* !< Audio Frequency configuration  16000 Hz       */
/* !< Audio Frequency configuration  11025 Hz       */
/* !< Audio Frequency configuration   8000 Hz       */
/* !< Audio Freq not specified. Register I2SDIV = 2 */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup I2S_LL_Exported_Macros I2S Exported Macros
  * @{
  */
/* * @defgroup I2S_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in I2S register
  * @param  __INSTANCE__ I2S Instance
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in I2S register
  * @param  __INSTANCE__ I2S Instance
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup I2S_LL_Exported_Functions I2S Exported Functions
  * @{
  */
/* * @defgroup I2S_LL_EF_Configuration Configuration
  * @{
  */
/* *
  * @brief  Select I2S mode and Enable I2S peripheral
  * @rmtoll I2SCFGR      I2SMOD        LL_I2S_Enable\n
  *         I2SCFGR      I2SE          LL_I2S_Enable
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable I2S peripheral
  * @rmtoll I2SCFGR      I2SE          LL_I2S_Disable
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if I2S peripheral is enabled
  * @rmtoll I2SCFGR      I2SE          LL_I2S_IsEnabled
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set I2S data frame length
  * @rmtoll I2SCFGR      DATLEN        LL_I2S_SetDataFormat\n
  *         I2SCFGR      CHLEN         LL_I2S_SetDataFormat
  * @param  SPIx SPI Instance
  * @param  DataFormat This parameter can be one of the following values:
  *         @arg @ref LL_I2S_DATAFORMAT_16B
  *         @arg @ref LL_I2S_DATAFORMAT_16B_EXTENDED
  *         @arg @ref LL_I2S_DATAFORMAT_24B
  *         @arg @ref LL_I2S_DATAFORMAT_32B
  * @retval None
  */
/* *
  * @brief  Get I2S data frame length
  * @rmtoll I2SCFGR      DATLEN        LL_I2S_GetDataFormat\n
  *         I2SCFGR      CHLEN         LL_I2S_GetDataFormat
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2S_DATAFORMAT_16B
  *         @arg @ref LL_I2S_DATAFORMAT_16B_EXTENDED
  *         @arg @ref LL_I2S_DATAFORMAT_24B
  *         @arg @ref LL_I2S_DATAFORMAT_32B
  */
/* *
  * @brief  Set I2S clock polarity
  * @rmtoll I2SCFGR      CKPOL         LL_I2S_SetClockPolarity
  * @param  SPIx SPI Instance
  * @param  ClockPolarity This parameter can be one of the following values:
  *         @arg @ref LL_I2S_POLARITY_LOW
  *         @arg @ref LL_I2S_POLARITY_HIGH
  * @retval None
  */
/* *
  * @brief  Get I2S clock polarity
  * @rmtoll I2SCFGR      CKPOL         LL_I2S_GetClockPolarity
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2S_POLARITY_LOW
  *         @arg @ref LL_I2S_POLARITY_HIGH
  */
/* *
  * @brief  Set I2S standard protocol
  * @rmtoll I2SCFGR      I2SSTD        LL_I2S_SetStandard\n
  *         I2SCFGR      PCMSYNC       LL_I2S_SetStandard
  * @param  SPIx SPI Instance
  * @param  Standard This parameter can be one of the following values:
  *         @arg @ref LL_I2S_STANDARD_PHILIPS
  *         @arg @ref LL_I2S_STANDARD_MSB
  *         @arg @ref LL_I2S_STANDARD_LSB
  *         @arg @ref LL_I2S_STANDARD_PCM_SHORT
  *         @arg @ref LL_I2S_STANDARD_PCM_LONG
  * @retval None
  */
/* *
  * @brief  Get I2S standard protocol
  * @rmtoll I2SCFGR      I2SSTD        LL_I2S_GetStandard\n
  *         I2SCFGR      PCMSYNC       LL_I2S_GetStandard
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2S_STANDARD_PHILIPS
  *         @arg @ref LL_I2S_STANDARD_MSB
  *         @arg @ref LL_I2S_STANDARD_LSB
  *         @arg @ref LL_I2S_STANDARD_PCM_SHORT
  *         @arg @ref LL_I2S_STANDARD_PCM_LONG
  */
/* *
  * @brief  Set I2S transfer mode
  * @rmtoll I2SCFGR      I2SCFG        LL_I2S_SetTransferMode
  * @param  SPIx SPI Instance
  * @param  Mode This parameter can be one of the following values:
  *         @arg @ref LL_I2S_MODE_SLAVE_TX
  *         @arg @ref LL_I2S_MODE_SLAVE_RX
  *         @arg @ref LL_I2S_MODE_MASTER_TX
  *         @arg @ref LL_I2S_MODE_MASTER_RX
  * @retval None
  */
/* *
  * @brief  Get I2S transfer mode
  * @rmtoll I2SCFGR      I2SCFG        LL_I2S_GetTransferMode
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2S_MODE_SLAVE_TX
  *         @arg @ref LL_I2S_MODE_SLAVE_RX
  *         @arg @ref LL_I2S_MODE_MASTER_TX
  *         @arg @ref LL_I2S_MODE_MASTER_RX
  */
/* *
  * @brief  Set I2S linear prescaler
  * @rmtoll I2SPR        I2SDIV        LL_I2S_SetPrescalerLinear
  * @param  SPIx SPI Instance
  * @param  PrescalerLinear Value between Min_Data=0x02 and Max_Data=0xFF
  * @retval None
  */
/* *
  * @brief  Get I2S linear prescaler
  * @rmtoll I2SPR        I2SDIV        LL_I2S_GetPrescalerLinear
  * @param  SPIx SPI Instance
  * @retval PrescalerLinear Value between Min_Data=0x02 and Max_Data=0xFF
  */
/* *
  * @brief  Set I2S parity prescaler
  * @rmtoll I2SPR        ODD           LL_I2S_SetPrescalerParity
  * @param  SPIx SPI Instance
  * @param  PrescalerParity This parameter can be one of the following values:
  *         @arg @ref LL_I2S_PRESCALER_PARITY_EVEN
  *         @arg @ref LL_I2S_PRESCALER_PARITY_ODD
  * @retval None
  */
/* *
  * @brief  Get I2S parity prescaler
  * @rmtoll I2SPR        ODD           LL_I2S_GetPrescalerParity
  * @param  SPIx SPI Instance
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_I2S_PRESCALER_PARITY_EVEN
  *         @arg @ref LL_I2S_PRESCALER_PARITY_ODD
  */
/* *
  * @brief  Enable the master clock ouput (Pin MCK)
  * @rmtoll I2SPR        MCKOE         LL_I2S_EnableMasterClock
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable the master clock ouput (Pin MCK)
  * @rmtoll I2SPR        MCKOE         LL_I2S_DisableMasterClock
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if the master clock ouput (Pin MCK) is enabled
  * @rmtoll I2SPR        MCKOE         LL_I2S_IsEnabledMasterClock
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable asynchronous start
  * @rmtoll I2SCFGR      ASTRTEN       LL_I2S_EnableAsyncStart
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable  asynchronous start
  * @rmtoll I2SCFGR      ASTRTEN       LL_I2S_DisableAsyncStart
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if asynchronous start is enabled
  * @rmtoll I2SCFGR      ASTRTEN       LL_I2S_IsEnabledAsyncStart
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* SPI_I2SCFGR_ASTRTEN */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EF_FLAG FLAG Management
  * @{
  */
/* *
  * @brief  Check if Rx buffer is not empty
  * @rmtoll SR           RXNE          LL_I2S_IsActiveFlag_RXNE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if Tx buffer is empty
  * @rmtoll SR           TXE           LL_I2S_IsActiveFlag_TXE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get busy flag
  * @rmtoll SR           BSY           LL_I2S_IsActiveFlag_BSY
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get overrun error flag
  * @rmtoll SR           OVR           LL_I2S_IsActiveFlag_OVR
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get underrun error flag
  * @rmtoll SR           UDR           LL_I2S_IsActiveFlag_UDR
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get frame format error flag
  * @rmtoll SR           FRE           LL_I2S_IsActiveFlag_FRE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get channel side flag.
  * @note   0: Channel Left has to be transmitted or has been received\n
  *         1: Channel Right has to be transmitted or has been received\n
  *         It has no significance in PCM mode.
  * @rmtoll SR           CHSIDE        LL_I2S_IsActiveFlag_CHSIDE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Clear overrun error flag
  * @rmtoll SR           OVR           LL_I2S_ClearFlag_OVR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Clear underrun error flag
  * @rmtoll SR           UDR           LL_I2S_ClearFlag_UDR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Clear frame format error flag
  * @rmtoll SR           FRE           LL_I2S_ClearFlag_FRE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EF_IT Interrupt Management
  * @{
  */
/* *
  * @brief  Enable error IT
  * @note   This bit controls the generation of an interrupt when an error condition occurs (OVR, UDR and FRE in I2S mode).
  * @rmtoll CR2          ERRIE         LL_I2S_EnableIT_ERR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Enable Rx buffer not empty IT
  * @rmtoll CR2          RXNEIE        LL_I2S_EnableIT_RXNE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Enable Tx buffer empty IT
  * @rmtoll CR2          TXEIE         LL_I2S_EnableIT_TXE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable error IT
  * @note   This bit controls the generation of an interrupt when an error condition occurs (OVR, UDR and FRE in I2S mode).
  * @rmtoll CR2          ERRIE         LL_I2S_DisableIT_ERR
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable Rx buffer not empty IT
  * @rmtoll CR2          RXNEIE        LL_I2S_DisableIT_RXNE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable Tx buffer empty IT
  * @rmtoll CR2          TXEIE         LL_I2S_DisableIT_TXE
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if ERR IT is enabled
  * @rmtoll CR2          ERRIE         LL_I2S_IsEnabledIT_ERR
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RXNE IT is enabled
  * @rmtoll CR2          RXNEIE        LL_I2S_IsEnabledIT_RXNE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if TXE IT is enabled
  * @rmtoll CR2          TXEIE         LL_I2S_IsEnabledIT_TXE
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EF_DMA DMA Management
  * @{
  */
/* *
  * @brief  Enable DMA Rx
  * @rmtoll CR2          RXDMAEN       LL_I2S_EnableDMAReq_RX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable DMA Rx
  * @rmtoll CR2          RXDMAEN       LL_I2S_DisableDMAReq_RX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if DMA Rx is enabled
  * @rmtoll CR2          RXDMAEN       LL_I2S_IsEnabledDMAReq_RX
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Enable DMA Tx
  * @rmtoll CR2          TXDMAEN       LL_I2S_EnableDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Disable DMA Tx
  * @rmtoll CR2          TXDMAEN       LL_I2S_DisableDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval None
  */
/* *
  * @brief  Check if DMA Tx is enabled
  * @rmtoll CR2          TXDMAEN       LL_I2S_IsEnabledDMAReq_TX
  * @param  SPIx SPI Instance
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EF_DATA DATA Management
  * @{
  */
/* *
  * @brief  Read 16-Bits in data register
  * @rmtoll DR           DR            LL_I2S_ReceiveData16
  * @param  SPIx SPI Instance
  * @retval RxData Value between Min_Data=0x0000 and Max_Data=0xFFFF
  */
/* *
  * @brief  Write 16-Bits in data register
  * @rmtoll DR           DR            LL_I2S_TransmitData16
  * @param  SPIx SPI Instance
  * @param  TxData Value between Min_Data=0x0000 and Max_Data=0xFFFF
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup I2S_LL_EF_Init Initialization and de-initialization functions
  * @{
  */
/* *
  * @brief  Set linear and parity prescaler.
  * @note   To calculate value of PrescalerLinear(I2SDIV[7:0] bits) and PrescalerParity(ODD bit)\n
  *         Check Audio frequency table and formulas inside Reference Manual (SPI/I2S).
  * @param  SPIx SPI Instance
  * @param  PrescalerLinear value: Min_Data=0x02 and Max_Data=0xFF.
  * @param  PrescalerParity This parameter can be one of the following values:
  *         @arg @ref LL_I2S_PRESCALER_PARITY_EVEN
  *         @arg @ref LL_I2S_PRESCALER_PARITY_ODD
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_I2S_ConfigPrescaler(mut SPIx: *mut SPI_TypeDef,
                                                mut PrescalerLinear: uint32_t,
                                                mut PrescalerParity:
                                                    uint32_t) {
    /* Check the I2S parameters */
    /* Write to SPIx I2SPR */
    ::core::ptr::write_volatile(&mut (*SPIx).I2SPR as *mut uint32_t,
                                (*SPIx).I2SPR &
                                    !((0xff as libc::c_uint) <<
                                          0 as libc::c_uint |
                                          (0x1 as libc::c_uint) <<
                                              8 as libc::c_uint) |
                                    (PrescalerLinear |
                                         PrescalerParity <<
                                             8 as libc::c_uint));
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* defined (SPI1) || defined (SPI2) || defined (SPI3) || defined(SPI4) || defined(SPI5) || defined(SPI6)  */
/* *
  * @}
  */
/* *
  * @}
  */
/* *
  * @}
  */
