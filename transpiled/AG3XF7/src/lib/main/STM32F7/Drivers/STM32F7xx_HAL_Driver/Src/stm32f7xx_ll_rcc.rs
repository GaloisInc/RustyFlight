use ::libc;
extern "C" {
    /* !< System Clock Frequency (Core Clock) */
    #[no_mangle]
    static AHBPrescTable: [uint8_t; 16];
    /* !< AHB prescalers table values */
    #[no_mangle]
    static APBPrescTable: [uint8_t; 8];
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_RCC_ClocksTypeDef {
    pub SYSCLK_Frequency: uint32_t,
    pub HCLK_Frequency: uint32_t,
    pub PCLK1_Frequency: uint32_t,
    pub PCLK2_Frequency: uint32_t,
}
#[inline]
unsafe extern "C" fn LL_RCC_HSE_DisableBypass() {
    let ref mut fresh0 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !((0x1 as libc::c_uint) <<
                                           18 as libc::c_uint)) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_RCC_HSI_Enable() {
    let ref mut fresh1 =
        (*((0x40000000 as
                libc::c_uint).wrapping_add(0x20000 as
                                               libc::c_uint).wrapping_add(0x3800
                                                                              as
                                                                              libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (0x1 as libc::c_uint) <<
                                         0 as libc::c_uint) as uint32_t as
                                    uint32_t);
}
#[inline]
unsafe extern "C" fn LL_RCC_HSI_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_RCC_HSI_SetCalibTrimming(mut Value: uint32_t) {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CR as
                                    *mut uint32_t,
                                (*((0x40000000 as
                                        libc::c_uint).wrapping_add(0x20000 as
                                                                       libc::c_uint).wrapping_add(0x3800
                                                                                                      as
                                                                                                      libc::c_uint)
                                       as *mut RCC_TypeDef)).CR &
                                    !((0x1f as libc::c_uint) <<
                                          3 as libc::c_uint) |
                                    Value << 3 as libc::c_uint);
}
#[inline]
unsafe extern "C" fn LL_RCC_LSE_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).BDCR &
                (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_RCC_LSI_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CSR &
                (0x1 as libc::c_uint) << 1 as libc::c_uint ==
                (0x1 as libc::c_uint) << 1 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetSysClkSource() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x3 as libc::c_uint) << 2 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetAHBPrescaler() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0xf as libc::c_uint) << 4 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetAPB1Prescaler() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x7 as libc::c_uint) << 10 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetAPB2Prescaler() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR &
               (0x7 as libc::c_uint) << 13 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetUSARTClockSource(mut USARTx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR2 & USARTx |
               USARTx << 16 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetUARTClockSource(mut UARTx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR2 & UARTx |
               UARTx << 16 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetI2CClockSource(mut I2Cx: uint32_t)
 -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR2 & I2Cx) >>
               16 as libc::c_uint | I2Cx;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetLPTIMClockSource(mut LPTIMx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR2 &
               (0x3 as libc::c_uint) << 24 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetSAIClockSource(mut SAIx: uint32_t)
 -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR1 & SAIx) >>
               16 as libc::c_uint | SAIx;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetSDMMCClockSource(mut SDMMCx: uint32_t)
 -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).DCKCFGR2 & SDMMCx) >>
               16 as libc::c_uint | SDMMCx;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetCK48MClockSource(mut CK48Mx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR2 & CK48Mx;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetRNGClockSource(mut RNGx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR2 & RNGx;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetUSBClockSource(mut USBx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR2 & USBx;
}
#[inline]
unsafe extern "C" fn LL_RCC_GetI2SClockSource(mut I2Sx: uint32_t)
 -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).CFGR & I2Sx;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLL_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 25 as libc::c_uint ==
                (0x1 as libc::c_uint) << 25 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLL_GetN() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).PLLCFGR &
                (0x1ff as libc::c_uint) << 6 as libc::c_uint) >>
               6 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLL_GetP() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLCFGR &
               (0x3 as libc::c_uint) << 16 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLL_GetQ() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLCFGR &
               (0xf as libc::c_uint) << 24 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLL_GetMainSource() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLCFGR &
               (0x1 as libc::c_uint) << 22 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLL_GetDivider() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLCFGR &
               (0x3f as libc::c_uint) << 0 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLI2S_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 27 as libc::c_uint ==
                (0x1 as libc::c_uint) << 27 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLI2S_GetN() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).PLLI2SCFGR &
                (0x1ff as libc::c_uint) << 6 as libc::c_uint) >>
               6 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLI2S_GetQ() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLI2SCFGR &
               (0xf as libc::c_uint) << 24 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLI2S_GetR() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLI2SCFGR &
               (0x7 as libc::c_uint) << 28 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLI2S_GetDIVQ() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR1 &
               (0x1f as libc::c_uint) << 0 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLSAI_IsReady() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).CR &
                (0x1 as libc::c_uint) << 29 as libc::c_uint ==
                (0x1 as libc::c_uint) << 29 as libc::c_uint) as libc::c_int as
               uint32_t;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLSAI_GetN() -> uint32_t {
    return ((*((0x40000000 as
                    libc::c_uint).wrapping_add(0x20000 as
                                                   libc::c_uint).wrapping_add(0x3800
                                                                                  as
                                                                                  libc::c_uint)
                   as *mut RCC_TypeDef)).PLLSAICFGR &
                (0x1ff as libc::c_uint) << 6 as libc::c_uint) >>
               6 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLSAI_GetQ() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLSAICFGR &
               (0xf as libc::c_uint) << 24 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLSAI_GetP() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).PLLSAICFGR &
               (0x3 as libc::c_uint) << 16 as libc::c_uint;
}
#[inline]
unsafe extern "C" fn LL_RCC_PLLSAI_GetDIVQ() -> uint32_t {
    return (*((0x40000000 as
                   libc::c_uint).wrapping_add(0x20000 as
                                                  libc::c_uint).wrapping_add(0x3800
                                                                                 as
                                                                                 libc::c_uint)
                  as *mut RCC_TypeDef)).DCKCFGR1 &
               (0x1f as libc::c_uint) << 8 as libc::c_uint;
}
/* SPDIFRX */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @addtogroup RCC_LL_Exported_Functions
  * @{
  */
/* * @addtogroup RCC_LL_EF_Init
  * @{
  */
/* *
  * @brief  Reset the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  *         - HSI ON and used as system clock source
  *         - HSE and PLL OFF
  *         - AHB, APB1 and APB2 prescaler set to 1.
  *         - CSS, MCO OFF
  *         - All interrupts disabled
  * @note   This function doesn't modify the configuration of the
  *         - Peripheral clocks
  *         - LSI, LSE and RTC clocks
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RCC registers are de-initialized
  *          - ERROR: not applicable
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_DeInit() -> ErrorStatus {
    let mut vl_mask: uint32_t = 0 as libc::c_uint;
    /* Set HSION bit */
    LL_RCC_HSI_Enable();
    /* Reset CFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, 0 as libc::c_uint);
    vl_mask = 0xffffffff as libc::c_uint;
    /* Reset HSEON, PLLSYSON bits */
    vl_mask &=
        !((0x1 as libc::c_uint) << 16 as libc::c_uint |
              (0x1 as libc::c_uint) << 18 as libc::c_uint |
              (0x1 as libc::c_uint) << 24 as libc::c_uint |
              (0x1 as libc::c_uint) << 19 as libc::c_uint);
    /* Reset PLLSAION bit */
    vl_mask &= !((0x1 as libc::c_uint) << 28 as libc::c_uint);
    /* Reset PLLI2SON bit */
    vl_mask &= !((0x1 as libc::c_uint) << 26 as libc::c_uint);
    /* Write new mask in CR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CR as
                                    *mut uint32_t, vl_mask);
    /* Set HSITRIM bits to the reset value*/
    LL_RCC_HSI_SetCalibTrimming(0x10 as libc::c_uint);
    /* Reset PLLCFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLCFGR as
                                    *mut uint32_t,
                                0x24003010 as libc::c_uint);
    /* Reset PLLI2SCFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLI2SCFGR
                                    as *mut uint32_t,
                                0x24003000 as libc::c_uint);
    /* Reset PLLSAICFGR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).PLLSAICFGR
                                    as *mut uint32_t,
                                0x24003000 as libc::c_uint);
    /* Reset HSEBYP bit */
    LL_RCC_HSE_DisableBypass();
    /* Disable all interrupts */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as
                                             libc::c_uint).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x3800
                                                                                                           as
                                                                                                           libc::c_uint)
                                            as *mut RCC_TypeDef)).CIR as
                                    *mut uint32_t, 0 as libc::c_uint);
    return SUCCESS;
}
/* *
  * @}
  */
/* * @addtogroup RCC_LL_EF_Get_Freq
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  *         and different peripheral clocks available on the device.
  * @note   If SYSCLK source is HSI, function returns values based on HSI_VALUE(**)
  * @note   If SYSCLK source is HSE, function returns values based on HSE_VALUE(***)
  * @note   If SYSCLK source is PLL, function returns values based on HSE_VALUE(***)
  *         or HSI_VALUE(**) multiplied/divided by the PLL factors.
  * @note   (**) HSI_VALUE is a constant defined in this file (default value
  *              16 MHz) but the real value may vary depending on the variations
  *              in voltage and temperature.
  * @note   (***) HSE_VALUE is a constant defined in this file (default value
  *               25 MHz), user has to ensure that HSE_VALUE is same as the real
  *               frequency of the crystal used. Otherwise, this function may
  *               have wrong result.
  * @note   The result of this function could be incorrect when using fractional
  *         value for HSE crystal.
  * @note   This function can be used by the user application to compute the
  *         baud-rate for the communication peripherals or configure other parameters.
  * @{
  */
/* *
  * @brief  Return the frequencies of different on chip clocks;  System, AHB, APB1 and APB2 buses clocks
  * @note   Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *         must be called to update structure fields. Otherwise, any
  *         configuration based on this function will be incorrect.
  * @param  RCC_Clocks pointer to a @ref LL_RCC_ClocksTypeDef structure which will hold the clocks frequencies
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetSystemClocksFreq(mut RCC_Clocks:
                                                        *mut LL_RCC_ClocksTypeDef) {
    /* Get SYSCLK frequency */
    (*RCC_Clocks).SYSCLK_Frequency = RCC_GetSystemClockFreq();
    /* HCLK clock frequency */
    (*RCC_Clocks).HCLK_Frequency =
        RCC_GetHCLKClockFreq((*RCC_Clocks).SYSCLK_Frequency);
    /* PCLK1 clock frequency */
    (*RCC_Clocks).PCLK1_Frequency =
        RCC_GetPCLK1ClockFreq((*RCC_Clocks).HCLK_Frequency);
    /* PCLK2 clock frequency */
    (*RCC_Clocks).PCLK2_Frequency =
        RCC_GetPCLK2ClockFreq((*RCC_Clocks).HCLK_Frequency);
}
/* *
  * @brief  Return USARTx clock frequency
  * @param  USARTxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USART1_CLKSOURCE
  *         @arg @ref LL_RCC_USART2_CLKSOURCE
  *         @arg @ref LL_RCC_USART3_CLKSOURCE
  *         @arg @ref LL_RCC_USART6_CLKSOURCE
  * @retval USART clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (HSI or LSE) is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetUSARTClockFreq(mut USARTxSource: uint32_t)
 -> uint32_t {
    let mut usart_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if USARTxSource == (0x3 as libc::c_uint) << 0 as libc::c_uint {
        /* USART1CLK clock frequency */
        match LL_RCC_GetUSARTClockSource(USARTxSource) {
            196609 => {
                /* USART1 Clock is System Clock */
                usart_frequency = RCC_GetSystemClockFreq()
            }
            196610 => {
                /* USART1 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    usart_frequency = 16000000 as libc::c_uint
                }
            }
            196611 => {
                /* USART1 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    usart_frequency = 32768 as libc::c_uint
                }
            }
            196608 | _ => {
                /* USART1 Clock is PCLK2 */
                usart_frequency =
                    RCC_GetPCLK2ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if USARTxSource == (0x3 as libc::c_uint) << 2 as libc::c_uint {
        /* USART2CLK clock frequency */
        match LL_RCC_GetUSARTClockSource(USARTxSource) {
            786436 => {
                /* USART2 Clock is System Clock */
                usart_frequency = RCC_GetSystemClockFreq()
            }
            786440 => {
                /* USART2 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    usart_frequency = 16000000 as libc::c_uint
                }
            }
            786444 => {
                /* USART2 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    usart_frequency = 32768 as libc::c_uint
                }
            }
            786432 | _ => {
                /* USART2 Clock is PCLK1 */
                usart_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if USARTxSource == (0x3 as libc::c_uint) << 10 as libc::c_uint {
        /* USART6CLK clock frequency */
        match LL_RCC_GetUSARTClockSource(USARTxSource) {
            201327616 => {
                /* USART6 Clock is System Clock */
                usart_frequency = RCC_GetSystemClockFreq()
            }
            201328640 => {
                /* USART6 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    usart_frequency = 16000000 as libc::c_uint
                }
            }
            201329664 => {
                /* USART6 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    usart_frequency = 32768 as libc::c_uint
                }
            }
            201326592 | _ => {
                /* USART6 Clock is PCLK2 */
                usart_frequency =
                    RCC_GetPCLK2ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if USARTxSource == (0x3 as libc::c_uint) << 4 as libc::c_uint {
        /* USART3CLK clock frequency */
        match LL_RCC_GetUSARTClockSource(USARTxSource) {
            3145744 => {
                /* USART3 Clock is System Clock */
                usart_frequency = RCC_GetSystemClockFreq()
            }
            3145760 => {
                /* USART3 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    usart_frequency = 16000000 as libc::c_uint
                }
            }
            3145776 => {
                /* USART3 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    usart_frequency = 32768 as libc::c_uint
                }
            }
            3145728 | _ => {
                /* USART3 Clock is PCLK1 */
                usart_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    }
    return usart_frequency;
}
/* *
  * @brief  Return UARTx clock frequency
  * @param  UARTxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_UART4_CLKSOURCE
  *         @arg @ref LL_RCC_UART5_CLKSOURCE
  *         @arg @ref LL_RCC_UART7_CLKSOURCE
  *         @arg @ref LL_RCC_UART8_CLKSOURCE
  * @retval UART clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (HSI or LSE) is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetUARTClockFreq(mut UARTxSource: uint32_t)
 -> uint32_t {
    let mut uart_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if UARTxSource == (0x3 as libc::c_uint) << 6 as libc::c_uint {
        /* UART4CLK clock frequency */
        match LL_RCC_GetUARTClockSource(UARTxSource) {
            12582976 => {
                /* UART4 Clock is System Clock */
                uart_frequency = RCC_GetSystemClockFreq()
            }
            12583040 => {
                /* UART4 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    uart_frequency = 16000000 as libc::c_uint
                }
            }
            12583104 => {
                /* UART4 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    uart_frequency = 32768 as libc::c_uint
                }
            }
            12582912 | _ => {
                /* UART4 Clock is PCLK1 */
                uart_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if UARTxSource == (0x3 as libc::c_uint) << 8 as libc::c_uint {
        /* UART5CLK clock frequency */
        match LL_RCC_GetUARTClockSource(UARTxSource) {
            50331904 => {
                /* UART5 Clock is System Clock */
                uart_frequency = RCC_GetSystemClockFreq()
            }
            50332160 => {
                /* UART5 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    uart_frequency = 16000000 as libc::c_uint
                }
            }
            50332416 => {
                /* UART5 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    uart_frequency = 32768 as libc::c_uint
                }
            }
            50331648 | _ => {
                /* UART5 Clock is PCLK1 */
                uart_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if UARTxSource == (0x3 as libc::c_uint) << 12 as libc::c_uint {
        /* UART7CLK clock frequency */
        match LL_RCC_GetUARTClockSource(UARTxSource) {
            805310464 => {
                /* UART7 Clock is System Clock */
                uart_frequency = RCC_GetSystemClockFreq()
            }
            805314560 => {
                /* UART7 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    uart_frequency = 16000000 as libc::c_uint
                }
            }
            805318656 => {
                /* UART7 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    uart_frequency = 32768 as libc::c_uint
                }
            }
            805306368 | _ => {
                /* UART7 Clock is PCLK1 */
                uart_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if UARTxSource == (0x3 as libc::c_uint) << 14 as libc::c_uint {
        /* UART8CLK clock frequency */
        match LL_RCC_GetUARTClockSource(UARTxSource) {
            3221241856 => {
                /* UART8 Clock is System Clock */
                uart_frequency = RCC_GetSystemClockFreq()
            }
            3221258240 => {
                /* UART8 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    uart_frequency = 16000000 as libc::c_uint
                }
            }
            3221274624 => {
                /* UART8 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    uart_frequency = 32768 as libc::c_uint
                }
            }
            3221225472 | _ => {
                /* UART8 Clock is PCLK1 */
                uart_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    }
    return uart_frequency;
}
/* *
  * @brief  Return I2Cx clock frequency
  * @param  I2CxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE (*)
  *
  *         (*) value not defined in all devices.
  * @retval I2C clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that HSI oscillator is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetI2CClockFreq(mut I2CxSource: uint32_t)
 -> uint32_t {
    let mut i2c_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if I2CxSource == (0x3 as libc::c_uint) << 16 as libc::c_uint {
        /* I2C1 CLK clock frequency */
        match LL_RCC_GetI2CClockSource(I2CxSource) {
            196609 => {
                /* I2C1 Clock is System Clock */
                i2c_frequency = RCC_GetSystemClockFreq()
            }
            196610 => {
                /* I2C1 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    i2c_frequency = 16000000 as libc::c_uint
                }
            }
            196608 | _ => {
                /* I2C1 Clock is PCLK1 */
                i2c_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if I2CxSource == (0x3 as libc::c_uint) << 18 as libc::c_uint {
        /* I2C2 CLK clock frequency */
        match LL_RCC_GetI2CClockSource(I2CxSource) {
            786436 => {
                /* I2C2 Clock is System Clock */
                i2c_frequency = RCC_GetSystemClockFreq()
            }
            786440 => {
                /* I2C2 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    i2c_frequency = 16000000 as libc::c_uint
                }
            }
            786432 | _ => {
                /* I2C2 Clock is PCLK1 */
                i2c_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    } else if I2CxSource == (0x3 as libc::c_uint) << 20 as libc::c_uint {
        /* I2C3 CLK clock frequency */
        match LL_RCC_GetI2CClockSource(I2CxSource) {
            3145744 => {
                /* I2C3 Clock is System Clock */
                i2c_frequency = RCC_GetSystemClockFreq()
            }
            3145760 => {
                /* I2C3 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    i2c_frequency = 16000000 as libc::c_uint
                }
            }
            3145728 | _ => {
                /* I2C3 Clock is PCLK1 */
                i2c_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    }
    /* I2C4 */
    return i2c_frequency;
}
/* DFSDM1_Channel0 */
/* *
  * @brief  Return I2Sx clock frequency
  * @param  I2SxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2S1_CLKSOURCE
  * @retval I2S clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that PLLI2S oscillator is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetI2SClockFreq(mut I2SxSource: uint32_t)
 -> uint32_t {
    let mut i2s_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if I2SxSource == (0x1 as libc::c_uint) << 23 as libc::c_uint {
        /* I2S1 CLK clock frequency */
        match LL_RCC_GetI2SClockSource(I2SxSource) {
            0 => {
                /* I2S1 Clock is PLLI2S */
                if LL_RCC_PLLI2S_IsReady() != 0 {
                    i2s_frequency = RCC_PLLI2S_GetFreqDomain_I2S()
                }
            }
            8388608 | _ => {
                /* I2S1 Clock is External clock */
                i2s_frequency = 12288000 as libc::c_uint
            }
        }
    }
    return i2s_frequency;
}
/* *
  * @brief  Return LPTIMx clock frequency
  * @param  LPTIMxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE
  * @retval LPTIM clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator (HSI, LSI or LSE) is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetLPTIMClockFreq(mut LPTIMxSource: uint32_t)
 -> uint32_t {
    let mut lptim_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if LPTIMxSource == (0x3 as libc::c_uint) << 24 as libc::c_uint {
        /* LPTIM1CLK clock frequency */
        match LL_RCC_GetLPTIMClockSource(LPTIMxSource) {
            16777216 => {
                /* LPTIM1 Clock is LSI Osc. */
                if LL_RCC_LSI_IsReady() != 0 {
                    lptim_frequency = 32000 as libc::c_uint
                }
            }
            33554432 => {
                /* LPTIM1 Clock is HSI Osc. */
                if LL_RCC_HSI_IsReady() != 0 {
                    lptim_frequency = 16000000 as libc::c_uint
                }
            }
            50331648 => {
                /* LPTIM1 Clock is LSE Osc. */
                if LL_RCC_LSE_IsReady() != 0 {
                    lptim_frequency = 32768 as libc::c_uint
                }
            }
            0 | _ => {
                /* LPTIM1 Clock is PCLK1 */
                lptim_frequency =
                    RCC_GetPCLK1ClockFreq(RCC_GetHCLKClockFreq(RCC_GetSystemClockFreq()))
            }
        }
    }
    return lptim_frequency;
}
/* *
  * @brief  Return SAIx clock frequency
  * @param  SAIxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE
  * @retval SAI clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that PLL is not ready
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NA indicates that external clock is used
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetSAIClockFreq(mut SAIxSource: uint32_t)
 -> uint32_t {
    let mut sai_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if SAIxSource == (0x3 as libc::c_uint) << 20 as libc::c_uint {
        /* SAI1CLK clock frequency */
        match LL_RCC_GetSAIClockSource(SAIxSource) {
            3145728 => {
                /* PLLSAI clock used as SAI1 clock source */
                if LL_RCC_PLLSAI_IsReady() != 0 {
                    sai_frequency = RCC_PLLSAI_GetFreqDomain_SAI()
                }
            }
            3145744 => {
                /* PLLI2S clock used as SAI1 clock source */
                if LL_RCC_PLLI2S_IsReady() != 0 {
                    sai_frequency = RCC_PLLI2S_GetFreqDomain_SAI()
                }
            }
            3145760 | _ => {
                /* External input clock used as SAI1 clock source */
                sai_frequency = 0xffffffff as libc::c_uint
            }
        }
    } else if SAIxSource == (0x3 as libc::c_uint) << 22 as libc::c_uint {
        /* SAI2CLK clock frequency */
        match LL_RCC_GetSAIClockSource(SAIxSource) {
            12582912 => {
                /* PLLSAI clock used as SAI2 clock source */
                if LL_RCC_PLLSAI_IsReady() != 0 {
                    sai_frequency = RCC_PLLSAI_GetFreqDomain_SAI()
                }
            }
            12582976 => {
                /* PLLI2S clock used as SAI2 clock source */
                if LL_RCC_PLLI2S_IsReady() != 0 {
                    sai_frequency = RCC_PLLI2S_GetFreqDomain_SAI()
                }
            }
            12583040 | _ => {
                /* External input clock used as SAI2 clock source */
                sai_frequency = 0xffffffff as libc::c_uint
            }
        }
    }
    return sai_frequency;
}
/* *
  * @brief  Return SDMMCx clock frequency
  * @param  SDMMCxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE
  *         @arg @ref LL_RCC_SDMMC2_CLKSOURCE (*)
  *
  *         (*) value not defined in all devices.
  * @retval SDMMC clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator PLL is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetSDMMCClockFreq(mut SDMMCxSource: uint32_t)
 -> uint32_t {
    let mut sdmmc_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    if SDMMCxSource == (0x1 as libc::c_uint) << 28 as libc::c_uint {
        /* SDMMC1CLK clock frequency */
        match LL_RCC_GetSDMMCClockSource(SDMMCxSource) {
            268435456 => {
                /* PLL48 clock used as SDMMC1 clock source */
                match LL_RCC_GetCK48MClockSource((0x1 as libc::c_uint) <<
                                                     27 as libc::c_uint) {
                    0 => {
                        /* PLL clock used as 48Mhz domain clock */
                        if LL_RCC_PLL_IsReady() != 0 {
                            sdmmc_frequency = RCC_PLL_GetFreqDomain_48M()
                        }
                    }
                    134217728 | _ => {
                        /* PLLSAI clock used as 48Mhz domain clock */
                        if LL_RCC_PLLSAI_IsReady() != 0 {
                            sdmmc_frequency = RCC_PLLSAI_GetFreqDomain_48M()
                        }
                    }
                }
            }
            268439552 | _ => {
                /* PLL clock used as SDMMC1 clock source */
                sdmmc_frequency = RCC_GetSystemClockFreq()
            }
        }
    } else {
        /* SDMMC2CLK clock frequency */
        match LL_RCC_GetSDMMCClockSource(SDMMCxSource) {
            536870912 => {
                /* PLL48 clock used as SDMMC2 clock source */
                match LL_RCC_GetCK48MClockSource((0x1 as libc::c_uint) <<
                                                     27 as libc::c_uint) {
                    0 => {
                        /* PLL clock used as 48Mhz domain clock */
                        if LL_RCC_PLL_IsReady() != 0 {
                            sdmmc_frequency = RCC_PLL_GetFreqDomain_48M()
                        }
                    }
                    134217728 | _ => {
                        /* PLLSAI clock used as 48Mhz domain clock */
                        if LL_RCC_PLLSAI_IsReady() != 0 {
                            sdmmc_frequency = RCC_PLLSAI_GetFreqDomain_48M()
                        }
                    }
                }
            }
            536879104 | _ => {
                /* PLL clock used as SDMMC2 clock source */
                sdmmc_frequency = RCC_GetSystemClockFreq()
            }
        }
    }
    /* SDMMC2 */
    return sdmmc_frequency;
}
/* *
  * @brief  Return RNGx clock frequency
  * @param  RNGxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_RNG_CLKSOURCE
  * @retval RNG clock frequency (in Hz)
  *         - @ref  LL_RCC_PERIPH_FREQUENCY_NO indicates that oscillator is not ready
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetRNGClockFreq(mut RNGxSource: uint32_t)
 -> uint32_t {
    let mut rng_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    /* RNGCLK clock frequency */
    match LL_RCC_GetRNGClockSource(RNGxSource) {
        0 => {
            /* PLL clock used as RNG clock source */
            if LL_RCC_PLL_IsReady() != 0 {
                rng_frequency = RCC_PLL_GetFreqDomain_48M()
            }
        }
        134217728 | _ => {
            /* PLLSAI clock used as RNG clock source */
            if LL_RCC_PLLSAI_IsReady() != 0 {
                rng_frequency = RCC_PLLSAI_GetFreqDomain_48M()
            }
        }
    }
    return rng_frequency;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_rcc.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of RCC LL module.
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
/* * @defgroup RCC_LL RCC
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* * @defgroup RCC_LL_Private_Variables RCC Private Variables
  * @{
  */
/* RCC_DCKCFGR1_PLLSAIDIVR */
/* *
  * @}
  */
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @defgroup RCC_LL_Private_Macros RCC Private Macros
  * @{
  */
/* *
  * @}
  */
/*USE_FULL_LL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup RCC_LL_Exported_Types RCC Exported Types
  * @{
  */
/* * @defgroup LL_ES_CLOCK_FREQ Clocks Frequency Structure
  * @{
  */
/* *
  * @brief  RCC Clocks Frequency Structure
  */
/* !< SYSCLK clock frequency */
/* !< HCLK clock frequency */
/* !< PCLK1 clock frequency */
/* !< PCLK2 clock frequency */
/* *
  * @}
  */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup RCC_LL_Exported_Constants RCC Exported Constants
  * @{
  */
/* * @defgroup RCC_LL_EC_OSC_VALUES Oscillator Values adaptation
  * @brief    Defines used to adapt values of different oscillators
  * @note     These values could be modified in the user environment according to 
  *           HW set-up.
  * @{
  */
/* HSE_VALUE */
/* HSI_VALUE */
/* LSE_VALUE */
/* LSI_VALUE */
/* EXTERNAL_CLOCK_VALUE */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_CLEAR_FLAG Clear Flags Defines
  * @brief    Flags defines which can be used with LL_RCC_WriteReg function
  * @{
  */
/* !< LSI Ready Interrupt Clear */
/* !< LSE Ready Interrupt Clear */
/* !< HSI Ready Interrupt Clear */
/* !< HSE Ready Interrupt Clear */
/* !< PLL Ready Interrupt Clear */
/* !< PLLI2S Ready Interrupt Clear */
/* !< PLLSAI Ready Interrupt Clear */
/* !< Clock Security System Interrupt Clear */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_GET_FLAG Get Flags Defines
  * @brief    Flags defines which can be used with LL_RCC_ReadReg function
  * @{
  */
/* !< LSI Ready Interrupt flag */
/* !< LSE Ready Interrupt flag */
/* !< HSI Ready Interrupt flag */
/* !< HSE Ready Interrupt flag */
/* !< PLL Ready Interrupt flag */
/* !< PLLI2S Ready Interrupt flag */
/* !< PLLSAI Ready Interrupt flag */
/* !< Clock Security System Interrupt flag */
/* !< Low-Power reset flag */
/* !< PIN reset flag */
/* !< POR/PDR reset flag */
/* !< Software Reset flag */
/* !< Independent Watchdog reset flag */
/* !< Window watchdog reset flag */
/* !< BOR reset flag */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_IT IT Defines
  * @brief    IT defines which can be used with LL_RCC_ReadReg and  LL_RCC_WriteReg functions
  * @{
  */
/* !< LSI Ready Interrupt Enable */
/* !< LSE Ready Interrupt Enable */
/* !< HSI Ready Interrupt Enable */
/* !< HSE Ready Interrupt Enable */
/* !< PLL Ready Interrupt Enable */
/* !< PLLI2S Ready Interrupt Enable */
/* !< PLLSAI Ready Interrupt Enable */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_LSEDRIVE  LSE oscillator drive capability
  * @{
  */
/* !< Xtal mode lower driving capability */
/* !< Xtal mode medium high driving capability */
/* !< Xtal mode medium low driving capability */
/* !< Xtal mode higher driving capability */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SYS_CLKSOURCE  System clock switch
  * @{
  */
/* !< HSI selection as system clock */
/* !< HSE selection as system clock */
/* !< PLL selection as system clock */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SYS_CLKSOURCE_STATUS  System clock switch status
  * @{
  */
/* !< HSI used as system clock */
/* !< HSE used as system clock */
/* !< PLL used as system clock */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SYSCLK_DIV  AHB prescaler
  * @{
  */
/* !< SYSCLK not divided */
/* !< SYSCLK divided by 2 */
/* !< SYSCLK divided by 4 */
/* !< SYSCLK divided by 8 */
/* !< SYSCLK divided by 16 */
/* !< SYSCLK divided by 64 */
/* !< SYSCLK divided by 128 */
/* !< SYSCLK divided by 256 */
/* !< SYSCLK divided by 512 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_APB1_DIV  APB low-speed prescaler (APB1)
  * @{
  */
/* !< HCLK not divided */
/* !< HCLK divided by 2 */
/* !< HCLK divided by 4 */
/* !< HCLK divided by 8 */
/* !< HCLK divided by 16 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_APB2_DIV  APB high-speed prescaler (APB2)
  * @{
  */
/* !< HCLK not divided */
/* !< HCLK divided by 2 */
/* !< HCLK divided by 4 */
/* !< HCLK divided by 8 */
/* !< HCLK divided by 16 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_MCOxSOURCE  MCO source selection
  * @{
  */
/* !< HSI selection as MCO1 source */
/* !< LSE selection as MCO1 source */
/* !< HSE selection as MCO1 source */
/* !< PLLCLK selection as MCO1 source */
/* !< SYSCLK selection as MCO2 source */
/* !< PLLI2S selection as MCO2 source */
/* !< HSE selection as MCO2 source */
/* !< PLLCLK selection as MCO2 source */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_MCOx_DIV  MCO prescaler
  * @{
  */
/* !< MCO1 not divided */
/* !< MCO1 divided by 2 */
/* !< MCO1 divided by 3 */
/* !< MCO1 divided by 4 */
/* !< MCO1 divided by 5 */
/* !< MCO2 not divided */
/* !< MCO2 divided by 2 */
/* !< MCO2 divided by 3 */
/* !< MCO2 divided by 4 */
/* !< MCO2 divided by 5 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_RTC_HSEDIV  HSE prescaler for RTC clock
  * @{
  */
/* !< HSE not divided */
/* !< HSE clock divided by 2 */
/* !< HSE clock divided by 3 */
/* !< HSE clock divided by 4 */
/* !< HSE clock divided by 5 */
/* !< HSE clock divided by 6 */
/* !< HSE clock divided by 7 */
/* !< HSE clock divided by 8 */
/* !< HSE clock divided by 9 */
/* !< HSE clock divided by 10 */
/* !< HSE clock divided by 11 */
/* !< HSE clock divided by 12 */
/* !< HSE clock divided by 13 */
/* !< HSE clock divided by 14 */
/* !< HSE clock divided by 15 */
/* !< HSE clock divided by 16 */
/* !< HSE clock divided by 17 */
/* !< HSE clock divided by 18 */
/* !< HSE clock divided by 19 */
/* !< HSE clock divided by 20 */
/* !< HSE clock divided by 21 */
/* !< HSE clock divided by 22 */
/* !< HSE clock divided by 23 */
/* !< HSE clock divided by 24 */
/* !< HSE clock divided by 25 */
/* !< HSE clock divided by 26 */
/* !< HSE clock divided by 27 */
/* !< HSE clock divided by 28 */
/* !< HSE clock divided by 29 */
/* !< HSE clock divided by 30 */
/* !< HSE clock divided by 31 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PERIPH_FREQUENCY Peripheral clock frequency
  * @{
  */
/* !< No clock enabled for the peripheral            */
/* !< Frequency cannot be provided as external clock */
/* *
  * @}
  */
/* USE_FULL_LL_DRIVER */
/* * @defgroup RCC_LL_EC_USARTx_CLKSOURCE  Peripheral USART clock source selection
  * @{
  */
/* !< PCLK2 clock used as USART1 clock source */
/* !< SYSCLK clock used as USART1 clock source */
/* !< HSI clock used as USART1 clock source */
/* !< LSE clock used as USART1 clock source */
/* !< PCLK1 clock used as USART2 clock source */
/* !< SYSCLK clock used as USART2 clock source */
/* !< HSI clock used as USART2 clock source */
/* !< LSE clock used as USART2 clock source */
/* !< PCLK1 clock used as USART3 clock source */
/* !< SYSCLK clock used as USART3 clock source */
/* !< HSI clock used as USART3 clock source */
/* !< LSE clock used as USART3 clock source */
/* !< PCLK2 clock used as USART6 clock source */
/* !< SYSCLK clock used as USART6 clock source */
/* !< HSI clock used as USART6 clock source */
/* !< LSE clock used as USART6 clock source */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_UARTx_CLKSOURCE  Peripheral UART clock source selection
  * @{
  */
/* !< PCLK1 clock used as UART4 clock source */
/* !< SYSCLK clock used as UART4 clock source */
/* !< HSI clock used as UART4 clock source */
/* !< LSE clock used as UART4 clock source */
/* !< PCLK1 clock used as UART5 clock source */
/* !< SYSCLK clock used as UART5 clock source */
/* !< HSI clock used as UART5 clock source */
/* !< LSE clock used as UART5 clock source */
/* !< PCLK1 clock used as UART7 clock source */
/* !< SYSCLK clock used as UART7 clock source */
/* !< HSI clock used as UART7 clock source */
/* !< LSE clock used as UART7 clock source */
/* !< PCLK1 clock used as UART8 clock source */
/* !< SYSCLK clock used as UART8 clock source */
/* !< HSI clock used as UART8 clock source */
/* !< LSE clock used as UART8 clock source */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_I2Cx_CLKSOURCE  Peripheral I2C clock source selection
  * @{
  */
/* !< PCLK1 clock used as I2C1 clock source */
/* !< SYSCLK clock used as I2C1 clock source */
/* !< HSI clock used as I2C1 clock source */
/* !< PCLK1 clock used as I2C2 clock source */
/* !< SYSCLK clock used as I2C2 clock source */
/* !< HSI clock used as I2C2 clock source */
/* !< PCLK1 clock used as I2C3 clock source */
/* !< SYSCLK clock used as I2C3 clock source */
/* !< HSI clock used as I2C3 clock source */
/* I2C4 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_LPTIM1_CLKSOURCE  Peripheral LPTIM clock source selection
  * @{
  */
/* !< PCLK1 clock used as LPTIM1 clock */
/* !< LSI oscillator clock used as LPTIM1 clock */
/* !< HSI oscillator clock used as LPTIM1 clock */
/* !< LSE oscillator clock used as LPTIM1 clock */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SAIx_CLKSOURCE  Peripheral SAI clock source selection
  * @{
  */
/* !< PLLSAI clock used as SAI1 clock source */
/* !< PLLI2S clock used as SAI1 clock source */
/* !< External pin clock used as SAI1 clock source */
/* RCC_SAI1SEL_PLLSRC_SUPPORT */
/* !< PLLSAI clock used as SAI2 clock source */
/* !< PLLI2S clock used as SAI2 clock source */
/* !< External pin clock used as SAI2 clock source */
/* RCC_SAI2SEL_PLLSRC_SUPPORT */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SDMMCx_CLKSOURCE  Peripheral SDMMC clock source selection
  * @{
  */
/* !< PLL 48M domain clock used as SDMMC1 clock */
/* !< System clock clock used as SDMMC1 clock */
/* !< PLL 48M domain clock used as SDMMC2 clock */
/* !< System clock clock used as SDMMC2 clock */
/* SDMMC2 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_RNG_CLKSOURCE  Peripheral RNG clock source selection
  * @{
  */
/* !< PLL clock used as RNG clock source */
/* !< PLLSAI clock used as RNG clock source */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_USB_CLKSOURCE  Peripheral USB clock source selection
  * @{
  */
/* !< PLL clock used as USB clock source */
/* !< PLLSAI1 clock used as USB clock source */
/* *
  * @}
  */
/* DSI */
/* CEC */
/* * @defgroup RCC_LL_EC_I2S1_CLKSOURCE  Peripheral I2S clock source selection
  * @{
  */
/* !< I2S oscillator clock used as I2S1 clock */
/* !< External pin clock used as I2S1 clock */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_CK48M_CLKSOURCE  Peripheral 48Mhz domain clock source selection
  * @{
  */
/* !< PLL oscillator clock used as 48Mhz domain clock */
/* !< PLLSAI oscillator clock used as 48Mhz domain clock */
/* *
  * @}
  */
/* DFSDM1_Channel0 */
/* * @defgroup RCC_LL_EC_USARTx  Peripheral USART get clock source
  * @{
  */
/* !< USART1 Clock source selection */
/* !< USART2 Clock source selection */
/* !< USART3 Clock source selection */
/* !< USART6 Clock source selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_UARTx  Peripheral UART get clock source
  * @{
  */
/* !< UART4 Clock source selection */
/* !< UART5 Clock source selection */
/* !< UART7 Clock source selection */
/* !< UART8 Clock source selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_I2Cx  Peripheral I2C get clock source
  * @{
  */
/* !< I2C1 Clock source selection */
/* !< I2C2 Clock source selection */
/* !< I2C3 Clock source selection */
/* I2C4 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_LPTIM1  Peripheral LPTIM get clock source
  * @{
  */
/* !< LPTIM1 Clock source selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SAIx  Peripheral SAI get clock source
  * @{
  */
/* !< SAI1 Clock source selection */
/* !< SAI2 Clock source selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_SDMMCx  Peripheral SDMMC get clock source
  * @{
  */
/* !< SDMMC1 Clock source selection */
/* !< SDMMC2 Clock source selection */
/* SDMMC2 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_CK48M  Peripheral CK48M get clock source
  * @{
  */
/* !< CK48M Domain clock source selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_RNG  Peripheral RNG get clock source
  * @{
  */
/* !< RNG Clock source selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_USB  Peripheral USB get clock source
  * @{
  */
/* !< USB Clock source selection */
/* *
  * @}
  */
/* CEC */
/* * @defgroup RCC_LL_EC_I2S1  Peripheral I2S get clock source
  * @{
  */
/* !< I2S Clock source selection */
/* *
  * @}
  */
/* DFSDM1_Channel0 */
/* DSI */
/* LTDC */
/* SPDIFRX */
/* * @defgroup RCC_LL_EC_RTC_CLKSOURCE  RTC clock source selection
  * @{
  */
/* !< No clock used as RTC clock */
/* !< LSE oscillator clock used as RTC clock */
/* !< LSI oscillator clock used as RTC clock */
/* !< HSE oscillator clock divided by HSE prescaler used as RTC clock */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_TIM_CLKPRESCALER  Timers clocks prescalers selection
  * @{
  */
/* !< Timers clock to twice PCLK */
/* !< Timers clock to four time PCLK */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLSOURCE  PLL, PLLI2S and PLLSAI entry clock source
  * @{
  */
/* !< HSI16 clock selected as PLL entry clock source */
/* !< HSE clock selected as PLL entry clock source */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLM_DIV  PLL, PLLI2S and PLLSAI division factor
  * @{
  */
/* !< PLL, PLLI2S and PLLSAI division factor by 2 */
/* !< PLL, PLLI2S and PLLSAI division factor by 3 */
/* !< PLL, PLLI2S and PLLSAI division factor by 4 */
/* !< PLL, PLLI2S and PLLSAI division factor by 5 */
/* !< PLL, PLLI2S and PLLSAI division factor by 6 */
/* !< PLL, PLLI2S and PLLSAI division factor by 7 */
/* !< PLL, PLLI2S and PLLSAI division factor by 8 */
/* !< PLL, PLLI2S and PLLSAI division factor by 9 */
/* !< PLL, PLLI2S and PLLSAI division factor by 10 */
/* !< PLL, PLLI2S and PLLSAI division factor by 11 */
/* !< PLL, PLLI2S and PLLSAI division factor by 12 */
/* !< PLL, PLLI2S and PLLSAI division factor by 13 */
/* !< PLL, PLLI2S and PLLSAI division factor by 14 */
/* !< PLL, PLLI2S and PLLSAI division factor by 15 */
/* !< PLL, PLLI2S and PLLSAI division factor by 16 */
/* !< PLL, PLLI2S and PLLSAI division factor by 17 */
/* !< PLL, PLLI2S and PLLSAI division factor by 18 */
/* !< PLL, PLLI2S and PLLSAI division factor by 19 */
/* !< PLL, PLLI2S and PLLSAI division factor by 20 */
/* !< PLL, PLLI2S and PLLSAI division factor by 21 */
/* !< PLL, PLLI2S and PLLSAI division factor by 22 */
/* !< PLL, PLLI2S and PLLSAI division factor by 23 */
/* !< PLL, PLLI2S and PLLSAI division factor by 24 */
/* !< PLL, PLLI2S and PLLSAI division factor by 25 */
/* !< PLL, PLLI2S and PLLSAI division factor by 26 */
/* !< PLL, PLLI2S and PLLSAI division factor by 27 */
/* !< PLL, PLLI2S and PLLSAI division factor by 28 */
/* !< PLL, PLLI2S and PLLSAI division factor by 29 */
/* !< PLL, PLLI2S and PLLSAI division factor by 30 */
/* !< PLL, PLLI2S and PLLSAI division factor by 31 */
/* !< PLL, PLLI2S and PLLSAI division factor by 32 */
/* !< PLL, PLLI2S and PLLSAI division factor by 33 */
/* !< PLL, PLLI2S and PLLSAI division factor by 34 */
/* !< PLL, PLLI2S and PLLSAI division factor by 35 */
/* !< PLL, PLLI2S and PLLSAI division factor by 36 */
/* !< PLL, PLLI2S and PLLSAI division factor by 37 */
/* !< PLL, PLLI2S and PLLSAI division factor by 38 */
/* !< PLL, PLLI2S and PLLSAI division factor by 39 */
/* !< PLL, PLLI2S and PLLSAI division factor by 40 */
/* !< PLL, PLLI2S and PLLSAI division factor by 41 */
/* !< PLL, PLLI2S and PLLSAI division factor by 42 */
/* !< PLL, PLLI2S and PLLSAI division factor by 43 */
/* !< PLL, PLLI2S and PLLSAI division factor by 44 */
/* !< PLL, PLLI2S and PLLSAI division factor by 45 */
/* !< PLL, PLLI2S and PLLSAI division factor by 46 */
/* !< PLL, PLLI2S and PLLSAI division factor by 47 */
/* !< PLL, PLLI2S and PLLSAI division factor by 48 */
/* !< PLL, PLLI2S and PLLSAI division factor by 49 */
/* !< PLL, PLLI2S and PLLSAI division factor by 50 */
/* !< PLL, PLLI2S and PLLSAI division factor by 51 */
/* !< PLL, PLLI2S and PLLSAI division factor by 52 */
/* !< PLL, PLLI2S and PLLSAI division factor by 53 */
/* !< PLL, PLLI2S and PLLSAI division factor by 54 */
/* !< PLL, PLLI2S and PLLSAI division factor by 55 */
/* !< PLL, PLLI2S and PLLSAI division factor by 56 */
/* !< PLL, PLLI2S and PLLSAI division factor by 57 */
/* !< PLL, PLLI2S and PLLSAI division factor by 58 */
/* !< PLL, PLLI2S and PLLSAI division factor by 59 */
/* !< PLL, PLLI2S and PLLSAI division factor by 60 */
/* !< PLL, PLLI2S and PLLSAI division factor by 61 */
/* !< PLL, PLLI2S and PLLSAI division factor by 62 */
/* !< PLL, PLLI2S and PLLSAI division factor by 63 */
/* *
  * @}
  */
/* RCC_PLLCFGR_PLLR */
/* * @defgroup RCC_LL_EC_PLLP_DIV  PLL division factor (PLLP)
  * @{
  */
/* !< Main PLL division factor for PLLP output by 2 */
/* !< Main PLL division factor for PLLP output by 4 */
/* !< Main PLL division factor for PLLP output by 6 */
/* !< Main PLL division factor for PLLP output by 8 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLQ_DIV  PLL division factor (PLLQ)
  * @{
  */
/* !< Main PLL division factor for PLLQ output by 2 */
/* !< Main PLL division factor for PLLQ output by 3 */
/* !< Main PLL division factor for PLLQ output by 4 */
/* !< Main PLL division factor for PLLQ output by 5 */
/* !< Main PLL division factor for PLLQ output by 6 */
/* !< Main PLL division factor for PLLQ output by 7 */
/* !< Main PLL division factor for PLLQ output by 8 */
/* !< Main PLL division factor for PLLQ output by 9 */
/* !< Main PLL division factor for PLLQ output by 10 */
/* !< Main PLL division factor for PLLQ output by 11 */
/* !< Main PLL division factor for PLLQ output by 12 */
/* !< Main PLL division factor for PLLQ output by 13 */
/* !< Main PLL division factor for PLLQ output by 14 */
/* !< Main PLL division factor for PLLQ output by 15 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLL_SPRE_SEL  PLL Spread Spectrum Selection
  * @{
  */
/* !< PLL center spread spectrum selection */
/* !< PLL down spread spectrum selection */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLI2SQ  PLLI2SQ division factor (PLLI2SQ)
  * @{
  */
/* !< PLLI2S division factor for PLLI2SQ output by 2 */
/* !< PLLI2S division factor for PLLI2SQ output by 3 */
/* !< PLLI2S division factor for PLLI2SQ output by 4 */
/* !< PLLI2S division factor for PLLI2SQ output by 5 */
/* !< PLLI2S division factor for PLLI2SQ output by 6 */
/* !< PLLI2S division factor for PLLI2SQ output by 7 */
/* !< PLLI2S division factor for PLLI2SQ output by 8 */
/* !< PLLI2S division factor for PLLI2SQ output by 9 */
/* !< PLLI2S division factor for PLLI2SQ output by 10 */
/* !< PLLI2S division factor for PLLI2SQ output by 11 */
/* !< PLLI2S division factor for PLLI2SQ output by 12 */
/* !< PLLI2S division factor for PLLI2SQ output by 13 */
/* !< PLLI2S division factor for PLLI2SQ output by 14 */
/* !< PLLI2S division factor for PLLI2SQ output by 15 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLI2SDIVQ  PLLI2SDIVQ division factor (PLLI2SDIVQ)
  * @{
  */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 1 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 2 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 3 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 4 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 5 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 6 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 7 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 8 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 9 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 10 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 11 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 12 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 13 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 14 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 15 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 16 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 17 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 18 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 19 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 20 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 21 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 22 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 23 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 24 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 25 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 26 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 27 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 28 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 29 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 30 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 31 */
/* !< PLLI2S division factor for PLLI2SDIVQ output by 32 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLI2SR  PLLI2SR division factor (PLLI2SR)
  * @{
  */
/* !< PLLI2S division factor for PLLI2SR output by 2 */
/* !< PLLI2S division factor for PLLI2SR output by 3 */
/* !< PLLI2S division factor for PLLI2SR output by 4 */
/* !< PLLI2S division factor for PLLI2SR output by 5 */
/* !< PLLI2S division factor for PLLI2SR output by 6 */
/* !< PLLI2S division factor for PLLI2SR output by 7 */
/* *
  * @}
  */
/* RCC_PLLI2SCFGR_PLLI2SP */
/* * @defgroup RCC_LL_EC_PLLSAIQ  PLLSAIQ division factor (PLLSAIQ)
  * @{
  */
/* !< PLLSAI division factor for PLLSAIQ output by 2 */
/* !< PLLSAI division factor for PLLSAIQ output by 3 */
/* !< PLLSAI division factor for PLLSAIQ output by 4 */
/* !< PLLSAI division factor for PLLSAIQ output by 5 */
/* !< PLLSAI division factor for PLLSAIQ output by 6 */
/* !< PLLSAI division factor for PLLSAIQ output by 7 */
/* !< PLLSAI division factor for PLLSAIQ output by 8 */
/* !< PLLSAI division factor for PLLSAIQ output by 9 */
/* !< PLLSAI division factor for PLLSAIQ output by 10 */
/* !< PLLSAI division factor for PLLSAIQ output by 11 */
/* !< PLLSAI division factor for PLLSAIQ output by 12 */
/* !< PLLSAI division factor for PLLSAIQ output by 13 */
/* !< PLLSAI division factor for PLLSAIQ output by 14 */
/* !< PLLSAI division factor for PLLSAIQ output by 15 */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EC_PLLSAIDIVQ  PLLSAIDIVQ division factor (PLLSAIDIVQ)
  * @{
  */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 1 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 2 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 3 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 4 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 5 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 6 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 7 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 8 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 9 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 10 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 11 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 12 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 13 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 14 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 15 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 16 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 17 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 18 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 19 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 20 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 21 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 22 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 23 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 24 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 25 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 26 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 27 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 28 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 29 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 30 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 31 */
/* !< PLLSAI division factor for PLLSAIDIVQ output by 32 */
/* *
  * @}
  */
/* RCC_PLLSAICFGR_PLLSAIR */
/* RCC_DCKCFGR1_PLLSAIDIVR */
/* * @defgroup RCC_LL_EC_PLLSAIP  PLLSAIP division factor (PLLSAIP)
  * @{
  */
/* !< PLLSAI division factor for PLLSAIP output by 2 */
/* !< PLLSAI division factor for PLLSAIP output by 4 */
/* !< PLLSAI division factor for PLLSAIP output by 6 */
/* !< PLLSAI division factor for PLLSAIP output by 8 */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* * @defgroup RCC_LL_Exported_Macros RCC Exported Macros
  * @{
  */
/* * @defgroup RCC_LL_EM_WRITE_READ Common Write and read registers Macros
  * @{
  */
/* *
  * @brief  Write a value in RCC register
  * @param  __REG__ Register to be written
  * @param  __VALUE__ Value to be written in the register
  * @retval None
  */
/* *
  * @brief  Read a value in RCC register
  * @param  __REG__ Register to be read
  * @retval Register value
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EM_CALC_FREQ Calculate frequencies
  * @{
  */
/* *
  * @brief  Helper macro to calculate the PLLCLK frequency on system domain
  * @note ex: @ref __LL_RCC_CALC_PLLCLK_FREQ (HSE_VALUE,@ref LL_RCC_PLL_GetDivider (),
  *             @ref LL_RCC_PLL_GetN (), @ref LL_RCC_PLL_GetP ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLM__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  __PLLN__ Between 50 and 432
  * @param  __PLLP__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLP_DIV_2
  *         @arg @ref LL_RCC_PLLP_DIV_4
  *         @arg @ref LL_RCC_PLLP_DIV_6
  *         @arg @ref LL_RCC_PLLP_DIV_8
  * @retval PLL clock frequency (in Hz)
  */
/* *
  * @brief  Helper macro to calculate the PLLCLK frequency used on 48M domain
  * @note ex: @ref __LL_RCC_CALC_PLLCLK_48M_FREQ (HSE_VALUE,@ref LL_RCC_PLL_GetDivider (),
  *             @ref LL_RCC_PLL_GetN (), @ref LL_RCC_PLL_GetQ ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLM__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  __PLLN__ Between 50 and 432
  * @param  __PLLQ__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLQ_DIV_2
  *         @arg @ref LL_RCC_PLLQ_DIV_3
  *         @arg @ref LL_RCC_PLLQ_DIV_4
  *         @arg @ref LL_RCC_PLLQ_DIV_5
  *         @arg @ref LL_RCC_PLLQ_DIV_6
  *         @arg @ref LL_RCC_PLLQ_DIV_7
  *         @arg @ref LL_RCC_PLLQ_DIV_8
  *         @arg @ref LL_RCC_PLLQ_DIV_9
  *         @arg @ref LL_RCC_PLLQ_DIV_10
  *         @arg @ref LL_RCC_PLLQ_DIV_11
  *         @arg @ref LL_RCC_PLLQ_DIV_12
  *         @arg @ref LL_RCC_PLLQ_DIV_13
  *         @arg @ref LL_RCC_PLLQ_DIV_14
  *         @arg @ref LL_RCC_PLLQ_DIV_15
  * @retval PLL clock frequency (in Hz)
  */
/* DSI */
/* *
  * @brief  Helper macro to calculate the PLLSAI frequency used for SAI1 and SAI2 domains
  * @note ex: @ref __LL_RCC_CALC_PLLSAI_SAI_FREQ (HSE_VALUE,@ref LL_RCC_PLL_GetDivider (),
  *             @ref LL_RCC_PLLSAI_GetN (), @ref LL_RCC_PLLSAI_GetQ (), @ref LL_RCC_PLLSAI_GetDIVQ ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLM__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  __PLLSAIN__ Between 50 and 432
  * @param  __PLLSAIQ__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_2
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_3
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_4
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_5
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_6
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_7
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_8
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_9
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_10
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_11
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_12
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_13
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_14
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_15
  * @param  __PLLSAIDIVQ__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_1
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_2
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_3
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_4
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_5
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_6
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_7
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_8
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_9
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_10
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_11
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_12
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_13
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_14
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_15
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_16
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_17
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_18
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_19
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_20
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_21
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_22
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_23
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_24
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_25
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_26
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_27
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_28
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_29
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_30
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_31
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_32
  * @retval PLLSAI clock frequency (in Hz)
  */
/* *
  * @brief  Helper macro to calculate the PLLSAI frequency used on 48Mhz domain
  * @note ex: @ref __LL_RCC_CALC_PLLSAI_48M_FREQ (HSE_VALUE,@ref LL_RCC_PLL_GetDivider (),
  *             @ref LL_RCC_PLLSAI_GetN (), @ref LL_RCC_PLLSAI_GetP ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLM__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  __PLLSAIN__ Between 50 and 432
  * @param  __PLLSAIP__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIP_DIV_2
  *         @arg @ref LL_RCC_PLLSAIP_DIV_4
  *         @arg @ref LL_RCC_PLLSAIP_DIV_6
  *         @arg @ref LL_RCC_PLLSAIP_DIV_8
  * @retval PLLSAI clock frequency (in Hz)
  */
/* LTDC */
/* *
  * @brief  Helper macro to calculate the PLLI2S frequency used for SAI1 and SAI2 domains
  * @note ex: @ref __LL_RCC_CALC_PLLI2S_SAI_FREQ (HSE_VALUE,@ref LL_RCC_PLL_GetDivider (),
  *             @ref LL_RCC_PLLI2S_GetN (), @ref LL_RCC_PLLI2S_GetQ (), @ref LL_RCC_PLLI2S_GetDIVQ ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLM__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  __PLLI2SN__ Between 50 and 432
  * @param  __PLLI2SQ__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_2
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_3
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_4
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_5
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_6
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_7
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_8
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_9
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_10
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_11
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_12
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_13
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_14
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_15
  * @param  __PLLI2SDIVQ__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_1
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_2
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_3
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_4
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_5
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_6
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_7
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_8
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_9
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_10
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_11
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_12
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_13
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_14
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_15
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_16
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_17
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_18
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_19
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_20
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_21
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_22
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_23
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_24
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_25
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_26
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_27
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_28
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_29
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_30
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_31
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_32
  * @retval PLLI2S clock frequency (in Hz)
  */
/* SPDIFRX */
/* *
  * @brief  Helper macro to calculate the PLLI2S frequency used for I2S domain
  * @note ex: @ref __LL_RCC_CALC_PLLI2S_I2S_FREQ (HSE_VALUE,@ref LL_RCC_PLL_GetDivider (),
  *             @ref LL_RCC_PLLI2S_GetN (), @ref LL_RCC_PLLI2S_GetR ());
  * @param  __INPUTFREQ__ PLL Input frequency (based on HSE/HSI)
  * @param  __PLLM__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  __PLLI2SN__ Between 50 and 432
  * @param  __PLLI2SR__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SR_DIV_2
  *         @arg @ref LL_RCC_PLLI2SR_DIV_3
  *         @arg @ref LL_RCC_PLLI2SR_DIV_4
  *         @arg @ref LL_RCC_PLLI2SR_DIV_5
  *         @arg @ref LL_RCC_PLLI2SR_DIV_6
  *         @arg @ref LL_RCC_PLLI2SR_DIV_7
  * @retval PLLI2S clock frequency (in Hz)
  */
/* *
  * @brief  Helper macro to calculate the HCLK frequency
  * @param  __SYSCLKFREQ__ SYSCLK frequency (based on HSE/HSI/PLLCLK)
  * @param  __AHBPRESCALER__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval HCLK clock frequency (in Hz)
  */
/* *
  * @brief  Helper macro to calculate the PCLK1 frequency (ABP1)
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB1PRESCALER__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval PCLK1 clock frequency (in Hz)
  */
/* *
  * @brief  Helper macro to calculate the PCLK2 frequency (ABP2)
  * @param  __HCLKFREQ__ HCLK frequency
  * @param  __APB2PRESCALER__ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  * @retval PCLK2 clock frequency (in Hz)
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported functions --------------------------------------------------------*/
/* * @defgroup RCC_LL_Exported_Functions RCC Exported Functions
  * @{
  */
/* * @defgroup RCC_LL_EF_HSE HSE
  * @{
  */
/* *
  * @brief  Enable the Clock Security System.
  * @rmtoll CR           CSSON         LL_RCC_HSE_EnableCSS
  * @retval None
  */
/* *
  * @brief  Enable HSE external oscillator (HSE Bypass)
  * @rmtoll CR           HSEBYP        LL_RCC_HSE_EnableBypass
  * @retval None
  */
/* *
  * @brief  Disable HSE external oscillator (HSE Bypass)
  * @rmtoll CR           HSEBYP        LL_RCC_HSE_DisableBypass
  * @retval None
  */
/* *
  * @brief  Enable HSE crystal oscillator (HSE ON)
  * @rmtoll CR           HSEON         LL_RCC_HSE_Enable
  * @retval None
  */
/* *
  * @brief  Disable HSE crystal oscillator (HSE ON)
  * @rmtoll CR           HSEON         LL_RCC_HSE_Disable
  * @retval None
  */
/* *
  * @brief  Check if HSE oscillator Ready
  * @rmtoll CR           HSERDY        LL_RCC_HSE_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_HSI HSI
  * @{
  */
/* *
  * @brief  Enable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Enable
  * @retval None
  */
/* *
  * @brief  Disable HSI oscillator
  * @rmtoll CR           HSION         LL_RCC_HSI_Disable
  * @retval None
  */
/* *
  * @brief  Check if HSI clock is ready
  * @rmtoll CR           HSIRDY        LL_RCC_HSI_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Get HSI Calibration value
  * @note When HSITRIM is written, HSICAL is updated with the sum of
  *       HSITRIM and the factory trim value
  * @rmtoll CR        HSICAL        LL_RCC_HSI_GetCalibration
  * @retval Between Min_Data = 0x00 and Max_Data = 0xFF
  */
/* *
  * @brief  Set HSI Calibration trimming
  * @note user-programmable trimming value that is added to the HSICAL
  * @note Default value is 16, which, when added to the HSICAL value,
  *       should trim the HSI to 16 MHz +/- 1 %
  * @rmtoll CR        HSITRIM       LL_RCC_HSI_SetCalibTrimming
  * @param  Value Between Min_Data = 0 and Max_Data = 31
  * @retval None
  */
/* *
  * @brief  Get HSI Calibration trimming
  * @rmtoll CR        HSITRIM       LL_RCC_HSI_GetCalibTrimming
  * @retval Between Min_Data = 0 and Max_Data = 31
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_LSE LSE
  * @{
  */
/* *
  * @brief  Enable  Low Speed External (LSE) crystal.
  * @rmtoll BDCR         LSEON         LL_RCC_LSE_Enable
  * @retval None
  */
/* *
  * @brief  Disable  Low Speed External (LSE) crystal.
  * @rmtoll BDCR         LSEON         LL_RCC_LSE_Disable
  * @retval None
  */
/* *
  * @brief  Enable external clock source (LSE bypass).
  * @rmtoll BDCR         LSEBYP        LL_RCC_LSE_EnableBypass
  * @retval None
  */
/* *
  * @brief  Disable external clock source (LSE bypass).
  * @rmtoll BDCR         LSEBYP        LL_RCC_LSE_DisableBypass
  * @retval None
  */
/* *
  * @brief  Set LSE oscillator drive capability
  * @note The oscillator is in Xtal mode when it is not in bypass mode.
  * @rmtoll BDCR         LSEDRV        LL_RCC_LSE_SetDriveCapability
  * @param  LSEDrive This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LSEDRIVE_LOW
  *         @arg @ref LL_RCC_LSEDRIVE_MEDIUMHIGH
  *         @arg @ref LL_RCC_LSEDRIVE_MEDIUMLOW
  *         @arg @ref LL_RCC_LSEDRIVE_HIGH
  * @retval None
  */
/* *
  * @brief  Get LSE oscillator drive capability
  * @rmtoll BDCR         LSEDRV        LL_RCC_LSE_GetDriveCapability
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_LSEDRIVE_LOW
  *         @arg @ref LL_RCC_LSEDRIVE_MEDIUMHIGH
  *         @arg @ref LL_RCC_LSEDRIVE_MEDIUMLOW
  *         @arg @ref LL_RCC_LSEDRIVE_HIGH
  */
/* *
  * @brief  Check if LSE oscillator Ready
  * @rmtoll BDCR         LSERDY        LL_RCC_LSE_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_LSI LSI
  * @{
  */
/* *
  * @brief  Enable LSI Oscillator
  * @rmtoll CSR          LSION         LL_RCC_LSI_Enable
  * @retval None
  */
/* *
  * @brief  Disable LSI Oscillator
  * @rmtoll CSR          LSION         LL_RCC_LSI_Disable
  * @retval None
  */
/* *
  * @brief  Check if LSI is Ready
  * @rmtoll CSR          LSIRDY        LL_RCC_LSI_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_System System
  * @{
  */
/* *
  * @brief  Configure the system clock source
  * @rmtoll CFGR         SW            LL_RCC_SetSysClkSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_PLL
  * @retval None
  */
/* *
  * @brief  Get the system clock source
  * @rmtoll CFGR         SWS           LL_RCC_GetSysClkSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSI
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_HSE
  *         @arg @ref LL_RCC_SYS_CLKSOURCE_STATUS_PLL
  */
/* *
  * @brief  Set AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_SetAHBPrescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  * @retval None
  */
/* *
  * @brief  Set APB1 prescaler
  * @rmtoll CFGR         PPRE1         LL_RCC_SetAPB1Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  * @retval None
  */
/* *
  * @brief  Set APB2 prescaler
  * @rmtoll CFGR         PPRE2         LL_RCC_SetAPB2Prescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  * @retval None
  */
/* *
  * @brief  Get AHB prescaler
  * @rmtoll CFGR         HPRE          LL_RCC_GetAHBPrescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SYSCLK_DIV_1
  *         @arg @ref LL_RCC_SYSCLK_DIV_2
  *         @arg @ref LL_RCC_SYSCLK_DIV_4
  *         @arg @ref LL_RCC_SYSCLK_DIV_8
  *         @arg @ref LL_RCC_SYSCLK_DIV_16
  *         @arg @ref LL_RCC_SYSCLK_DIV_64
  *         @arg @ref LL_RCC_SYSCLK_DIV_128
  *         @arg @ref LL_RCC_SYSCLK_DIV_256
  *         @arg @ref LL_RCC_SYSCLK_DIV_512
  */
/* *
  * @brief  Get APB1 prescaler
  * @rmtoll CFGR         PPRE1         LL_RCC_GetAPB1Prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_APB1_DIV_1
  *         @arg @ref LL_RCC_APB1_DIV_2
  *         @arg @ref LL_RCC_APB1_DIV_4
  *         @arg @ref LL_RCC_APB1_DIV_8
  *         @arg @ref LL_RCC_APB1_DIV_16
  */
/* *
  * @brief  Get APB2 prescaler
  * @rmtoll CFGR         PPRE2         LL_RCC_GetAPB2Prescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_APB2_DIV_1
  *         @arg @ref LL_RCC_APB2_DIV_2
  *         @arg @ref LL_RCC_APB2_DIV_4
  *         @arg @ref LL_RCC_APB2_DIV_8
  *         @arg @ref LL_RCC_APB2_DIV_16
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_MCO MCO
  * @{
  */
/* *
  * @brief  Configure MCOx
  * @rmtoll CFGR         MCO1          LL_RCC_ConfigMCO\n
  *         CFGR         MCO1PRE       LL_RCC_ConfigMCO\n
  *         CFGR         MCO2          LL_RCC_ConfigMCO\n
  *         CFGR         MCO2PRE       LL_RCC_ConfigMCO
  * @param  MCOxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1SOURCE_HSI
  *         @arg @ref LL_RCC_MCO1SOURCE_LSE
  *         @arg @ref LL_RCC_MCO1SOURCE_HSE
  *         @arg @ref LL_RCC_MCO1SOURCE_PLLCLK
  *         @arg @ref LL_RCC_MCO2SOURCE_SYSCLK
  *         @arg @ref LL_RCC_MCO2SOURCE_PLLI2S
  *         @arg @ref LL_RCC_MCO2SOURCE_HSE
  *         @arg @ref LL_RCC_MCO2SOURCE_PLLCLK
  * @param  MCOxPrescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_MCO1_DIV_1
  *         @arg @ref LL_RCC_MCO1_DIV_2
  *         @arg @ref LL_RCC_MCO1_DIV_3
  *         @arg @ref LL_RCC_MCO1_DIV_4
  *         @arg @ref LL_RCC_MCO1_DIV_5
  *         @arg @ref LL_RCC_MCO2_DIV_1
  *         @arg @ref LL_RCC_MCO2_DIV_2
  *         @arg @ref LL_RCC_MCO2_DIV_3
  *         @arg @ref LL_RCC_MCO2_DIV_4
  *         @arg @ref LL_RCC_MCO2_DIV_5
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_Peripheral_Clock_Source Peripheral Clock Source
  * @{
  */
/* *
  * @brief  Configure USARTx clock source
  * @rmtoll DCKCFGR2        USART1SEL     LL_RCC_SetUSARTClockSource\n
  *         DCKCFGR2        USART2SEL     LL_RCC_SetUSARTClockSource\n
  *         DCKCFGR2        USART3SEL     LL_RCC_SetUSARTClockSource\n
  *         DCKCFGR2        USART6SEL     LL_RCC_SetUSARTClockSource
  * @param  USARTxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_PCLK2
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_PCLK2
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_LSE
  * @retval None
  */
/* *
  * @brief  Configure UARTx clock source
  * @rmtoll DCKCFGR2        UART4SEL      LL_RCC_SetUARTClockSource\n
  *         DCKCFGR2        UART5SEL      LL_RCC_SetUARTClockSource\n
  *         DCKCFGR2        UART7SEL      LL_RCC_SetUARTClockSource\n
  *         DCKCFGR2        UART8SEL      LL_RCC_SetUARTClockSource
  * @param  UARTxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_LSE
  * @retval None
  */
/* *
  * @brief  Configure I2Cx clock source
  * @rmtoll DCKCFGR2        I2C1SEL       LL_RCC_SetI2CClockSource\n
  *         DCKCFGR2        I2C2SEL       LL_RCC_SetI2CClockSource\n
  *         DCKCFGR2        I2C3SEL       LL_RCC_SetI2CClockSource\n
  *         DCKCFGR2        I2C4SEL       LL_RCC_SetI2CClockSource
  * @param  I2CxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE_PCLK1 (*)
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE_SYSCLK (*)
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE_HSI (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
/* *
  * @brief  Configure LPTIMx clock source
  * @rmtoll DCKCFGR2        LPTIM1SEL     LL_RCC_SetLPTIMClockSource
  * @param  LPTIMxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSE
  * @retval None
  */
/* *
  * @brief  Configure SAIx clock source
  * @rmtoll DCKCFGR1        SAI1SEL       LL_RCC_SetSAIClockSource\n
  *         DCKCFGR1        SAI2SEL       LL_RCC_SetSAIClockSource
  * @param  SAIxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PLLSAI
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PLLI2S
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PIN
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PLLSRC (*)
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PLLSAI
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PLLI2S
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PIN
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PLLSRC (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
/* *
  * @brief  Configure SDMMC clock source
  * @rmtoll DCKCFGR2        SDMMC1SEL      LL_RCC_SetSDMMCClockSource\n
  *         DCKCFGR2        SDMMC2SEL      LL_RCC_SetSDMMCClockSource
  * @param  SDMMCxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE_PLL48CLK
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_SDMMC2_CLKSOURCE_PLL48CLK (*)
  *         @arg @ref LL_RCC_SDMMC2_CLKSOURCE_SYSCLK (*)
  *
  *         (*) value not defined in all devices.
  * @retval None
  */
/* *
  * @brief  Configure 48Mhz domain clock source
  * @rmtoll DCKCFGR2        CK48MSEL      LL_RCC_SetCK48MClockSource
  * @param  CK48MxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_CK48M_CLKSOURCE_PLL
  *         @arg @ref LL_RCC_CK48M_CLKSOURCE_PLLSAI
  * @retval None
  */
/* *
  * @brief  Configure RNG clock source
  * @rmtoll DCKCFGR2        CK48MSEL      LL_RCC_SetRNGClockSource
  * @param  RNGxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_RNG_CLKSOURCE_PLL
  *         @arg @ref LL_RCC_RNG_CLKSOURCE_PLLSAI
  * @retval None
  */
/* *
  * @brief  Configure USB clock source
  * @rmtoll DCKCFGR2        CK48MSEL      LL_RCC_SetUSBClockSource
  * @param  USBxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLLSAI
  * @retval None
  */
/* CEC */
/* *
  * @brief  Configure I2S clock source
  * @rmtoll CFGR         I2SSRC        LL_RCC_SetI2SClockSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2S1_CLKSOURCE_PLLI2S
  *         @arg @ref LL_RCC_I2S1_CLKSOURCE_PIN
  * @retval None
  */
/* DSI */
/* DFSDM1_Channel0 */
/* *
  * @brief  Get USARTx clock source
  * @rmtoll DCKCFGR2        USART1SEL     LL_RCC_GetUSARTClockSource\n
  *         DCKCFGR2        USART2SEL     LL_RCC_GetUSARTClockSource\n
  *         DCKCFGR2        USART3SEL     LL_RCC_GetUSARTClockSource\n
  *         DCKCFGR2        USART6SEL     LL_RCC_GetUSARTClockSource
  * @param  USARTx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USART1_CLKSOURCE
  *         @arg @ref LL_RCC_USART2_CLKSOURCE
  *         @arg @ref LL_RCC_USART3_CLKSOURCE
  *         @arg @ref LL_RCC_USART6_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_PCLK2
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART1_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART2_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART3_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_PCLK2
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_USART6_CLKSOURCE_LSE
  */
/* *
  * @brief  Get UARTx clock source
  * @rmtoll DCKCFGR2        UART4SEL      LL_RCC_GetUARTClockSource\n
  *         DCKCFGR2        UART5SEL      LL_RCC_GetUARTClockSource\n
  *         DCKCFGR2        UART7SEL      LL_RCC_GetUARTClockSource\n
  *         DCKCFGR2        UART8SEL      LL_RCC_GetUARTClockSource
  * @param  UARTx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_UART4_CLKSOURCE
  *         @arg @ref LL_RCC_UART5_CLKSOURCE
  *         @arg @ref LL_RCC_UART7_CLKSOURCE
  *         @arg @ref LL_RCC_UART8_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART4_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART5_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART7_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_UART8_CLKSOURCE_LSE
  */
/* *
  * @brief  Get I2Cx clock source
  * @rmtoll DCKCFGR2        I2C1SEL       LL_RCC_GetI2CClockSource\n
  *         DCKCFGR2        I2C2SEL       LL_RCC_GetI2CClockSource\n
  *         DCKCFGR2        I2C3SEL       LL_RCC_GetI2CClockSource\n
  *         DCKCFGR2        I2C4SEL       LL_RCC_GetI2CClockSource
  * @param  I2Cx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE (*)
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2C1_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2C2_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_I2C3_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE_PCLK1 (*)
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE_SYSCLK (*)
  *         @arg @ref LL_RCC_I2C4_CLKSOURCE_HSI (*)
  *
  *         (*) value not defined in all devices.
 */
/* *
  * @brief  Get LPTIMx clock source
  * @rmtoll DCKCFGR2        LPTIM1SEL     LL_RCC_GetLPTIMClockSource
  * @param  LPTIMx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_PCLK1
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_HSI
  *         @arg @ref LL_RCC_LPTIM1_CLKSOURCE_LSE
  */
/* *
  * @brief  Get SAIx clock source
  * @rmtoll DCKCFGR1        SAI1SEL       LL_RCC_GetSAIClockSource\n
  *         DCKCFGR1        SAI2SEL       LL_RCC_GetSAIClockSource
  * @param  SAIx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PLLSAI
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PLLI2S
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PIN
  *         @arg @ref LL_RCC_SAI1_CLKSOURCE_PLLSRC (*)
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PLLSAI
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PLLI2S
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PIN
  *         @arg @ref LL_RCC_SAI2_CLKSOURCE_PLLSRC (*)
  *
  *         (*) value not defined in all devices.
  */
/* *
  * @brief  Get SDMMCx clock source
  * @rmtoll DCKCFGR2        SDMMC1SEL      LL_RCC_GetSDMMCClockSource\n
  *         DCKCFGR2        SDMMC2SEL      LL_RCC_GetSDMMCClockSource
  * @param  SDMMCx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE (*)
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE_PLL48CLK
  *         @arg @ref LL_RCC_SDMMC1_CLKSOURCE_SYSCLK
  *         @arg @ref LL_RCC_SDMMC2_CLKSOURCE_PLL48CLK (*)
  *         @arg @ref LL_RCC_SDMMC2_CLKSOURCE_SYSCLK (*)
  *
  *         (*) value not defined in all devices.
  */
/* *
  * @brief  Get 48Mhz domain clock source
  * @rmtoll DCKCFGR2        CK48MSEL      LL_RCC_GetCK48MClockSource
  * @param  CK48Mx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_CK48M_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_CK48M_CLKSOURCE_PLL
  *         @arg @ref LL_RCC_CK48M_CLKSOURCE_PLLSAI
  */
/* *
  * @brief  Get RNGx clock source
  * @rmtoll DCKCFGR2        CK48MSEL      LL_RCC_GetRNGClockSource
  * @param  RNGx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_RNG_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_RNG_CLKSOURCE_PLL
  *         @arg @ref LL_RCC_RNG_CLKSOURCE_PLLSAI
  */
/* *
  * @brief  Get USBx clock source
  * @rmtoll DCKCFGR2        CK48MSEL      LL_RCC_GetUSBClockSource
  * @param  USBx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLL
  *         @arg @ref LL_RCC_USB_CLKSOURCE_PLLSAI
  */
/* CEC */
/* *
  * @brief  Get I2S Clock Source
  * @rmtoll CFGR         I2SSRC        LL_RCC_GetI2SClockSource
  * @param  I2Sx This parameter can be one of the following values:
  *         @arg @ref LL_RCC_I2S1_CLKSOURCE
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_I2S1_CLKSOURCE_PLLI2S
  *         @arg @ref LL_RCC_I2S1_CLKSOURCE_PIN
  */
/* DFSDM1_Channel0 */
/* DSI */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_RTC RTC
  * @{
  */
/* *
  * @brief  Set RTC Clock Source
  * @note Once the RTC clock source has been selected, it cannot be changed anymore unless
  *       the Backup domain is reset, or unless a failure is detected on LSE (LSECSSD is
  *       set). The BDRST bit can be used to reset them.
  * @rmtoll BDCR         RTCSEL        LL_RCC_SetRTCClockSource
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_NONE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_HSE
  * @retval None
  */
/* *
  * @brief  Get RTC Clock Source
  * @rmtoll BDCR         RTCSEL        LL_RCC_GetRTCClockSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_NONE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSE
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_LSI
  *         @arg @ref LL_RCC_RTC_CLKSOURCE_HSE
  */
/* *
  * @brief  Enable RTC
  * @rmtoll BDCR         RTCEN         LL_RCC_EnableRTC
  * @retval None
  */
/* *
  * @brief  Disable RTC
  * @rmtoll BDCR         RTCEN         LL_RCC_DisableRTC
  * @retval None
  */
/* *
  * @brief  Check if RTC has been enabled or not
  * @rmtoll BDCR         RTCEN         LL_RCC_IsEnabledRTC
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Force the Backup domain reset
  * @rmtoll BDCR         BDRST         LL_RCC_ForceBackupDomainReset
  * @retval None
  */
/* *
  * @brief  Release the Backup domain reset
  * @rmtoll BDCR         BDRST         LL_RCC_ReleaseBackupDomainReset
  * @retval None
  */
/* *
  * @brief  Set HSE Prescalers for RTC Clock
  * @rmtoll CFGR         RTCPRE        LL_RCC_SetRTC_HSEPrescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_RTC_NOCLOCK
  *         @arg @ref LL_RCC_RTC_HSE_DIV_2
  *         @arg @ref LL_RCC_RTC_HSE_DIV_3
  *         @arg @ref LL_RCC_RTC_HSE_DIV_4
  *         @arg @ref LL_RCC_RTC_HSE_DIV_5
  *         @arg @ref LL_RCC_RTC_HSE_DIV_6
  *         @arg @ref LL_RCC_RTC_HSE_DIV_7
  *         @arg @ref LL_RCC_RTC_HSE_DIV_8
  *         @arg @ref LL_RCC_RTC_HSE_DIV_9
  *         @arg @ref LL_RCC_RTC_HSE_DIV_10
  *         @arg @ref LL_RCC_RTC_HSE_DIV_11
  *         @arg @ref LL_RCC_RTC_HSE_DIV_12
  *         @arg @ref LL_RCC_RTC_HSE_DIV_13
  *         @arg @ref LL_RCC_RTC_HSE_DIV_14
  *         @arg @ref LL_RCC_RTC_HSE_DIV_15
  *         @arg @ref LL_RCC_RTC_HSE_DIV_16
  *         @arg @ref LL_RCC_RTC_HSE_DIV_17
  *         @arg @ref LL_RCC_RTC_HSE_DIV_18
  *         @arg @ref LL_RCC_RTC_HSE_DIV_19
  *         @arg @ref LL_RCC_RTC_HSE_DIV_20
  *         @arg @ref LL_RCC_RTC_HSE_DIV_21
  *         @arg @ref LL_RCC_RTC_HSE_DIV_22
  *         @arg @ref LL_RCC_RTC_HSE_DIV_23
  *         @arg @ref LL_RCC_RTC_HSE_DIV_24
  *         @arg @ref LL_RCC_RTC_HSE_DIV_25
  *         @arg @ref LL_RCC_RTC_HSE_DIV_26
  *         @arg @ref LL_RCC_RTC_HSE_DIV_27
  *         @arg @ref LL_RCC_RTC_HSE_DIV_28
  *         @arg @ref LL_RCC_RTC_HSE_DIV_29
  *         @arg @ref LL_RCC_RTC_HSE_DIV_30
  *         @arg @ref LL_RCC_RTC_HSE_DIV_31
  * @retval None
  */
/* *
  * @brief  Get HSE Prescalers for RTC Clock
  * @rmtoll CFGR         RTCPRE        LL_RCC_GetRTC_HSEPrescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_RTC_NOCLOCK
  *         @arg @ref LL_RCC_RTC_HSE_DIV_2
  *         @arg @ref LL_RCC_RTC_HSE_DIV_3
  *         @arg @ref LL_RCC_RTC_HSE_DIV_4
  *         @arg @ref LL_RCC_RTC_HSE_DIV_5
  *         @arg @ref LL_RCC_RTC_HSE_DIV_6
  *         @arg @ref LL_RCC_RTC_HSE_DIV_7
  *         @arg @ref LL_RCC_RTC_HSE_DIV_8
  *         @arg @ref LL_RCC_RTC_HSE_DIV_9
  *         @arg @ref LL_RCC_RTC_HSE_DIV_10
  *         @arg @ref LL_RCC_RTC_HSE_DIV_11
  *         @arg @ref LL_RCC_RTC_HSE_DIV_12
  *         @arg @ref LL_RCC_RTC_HSE_DIV_13
  *         @arg @ref LL_RCC_RTC_HSE_DIV_14
  *         @arg @ref LL_RCC_RTC_HSE_DIV_15
  *         @arg @ref LL_RCC_RTC_HSE_DIV_16
  *         @arg @ref LL_RCC_RTC_HSE_DIV_17
  *         @arg @ref LL_RCC_RTC_HSE_DIV_18
  *         @arg @ref LL_RCC_RTC_HSE_DIV_19
  *         @arg @ref LL_RCC_RTC_HSE_DIV_20
  *         @arg @ref LL_RCC_RTC_HSE_DIV_21
  *         @arg @ref LL_RCC_RTC_HSE_DIV_22
  *         @arg @ref LL_RCC_RTC_HSE_DIV_23
  *         @arg @ref LL_RCC_RTC_HSE_DIV_24
  *         @arg @ref LL_RCC_RTC_HSE_DIV_25
  *         @arg @ref LL_RCC_RTC_HSE_DIV_26
  *         @arg @ref LL_RCC_RTC_HSE_DIV_27
  *         @arg @ref LL_RCC_RTC_HSE_DIV_28
  *         @arg @ref LL_RCC_RTC_HSE_DIV_29
  *         @arg @ref LL_RCC_RTC_HSE_DIV_30
  *         @arg @ref LL_RCC_RTC_HSE_DIV_31
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_TIM_CLOCK_PRESCALER TIM
  * @{
  */
/* *
  * @brief  Set Timers Clock Prescalers
  * @rmtoll DCKCFGR1         TIMPRE        LL_RCC_SetTIMPrescaler
  * @param  Prescaler This parameter can be one of the following values:
  *         @arg @ref LL_RCC_TIM_PRESCALER_TWICE
  *         @arg @ref LL_RCC_TIM_PRESCALER_FOUR_TIMES
  * @retval None
  */
/* *
  * @brief  Get Timers Clock Prescalers
  * @rmtoll DCKCFGR1         TIMPRE        LL_RCC_GetTIMPrescaler
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_TIM_PRESCALER_TWICE
  *         @arg @ref LL_RCC_TIM_PRESCALER_FOUR_TIMES
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_PLL PLL
  * @{
  */
/* *
  * @brief  Enable PLL
  * @rmtoll CR           PLLON         LL_RCC_PLL_Enable
  * @retval None
  */
/* *
  * @brief  Disable PLL
  * @note Cannot be disabled if the PLL clock is used as the system clock
  * @rmtoll CR           PLLON         LL_RCC_PLL_Disable
  * @retval None
  */
/* *
  * @brief  Check if PLL Ready
  * @rmtoll CR           PLLRDY        LL_RCC_PLL_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Configure PLL used for SYSCLK Domain
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLP can be written only when PLL is disabled
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLL_ConfigDomain_SYS\n
  *         PLLCFGR      PLLM          LL_RCC_PLL_ConfigDomain_SYS\n
  *         PLLCFGR      PLLN          LL_RCC_PLL_ConfigDomain_SYS\n
  *         PLLCFGR      PLLP          LL_RCC_PLL_ConfigDomain_SYS
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLP This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLP_DIV_2
  *         @arg @ref LL_RCC_PLLP_DIV_4
  *         @arg @ref LL_RCC_PLLP_DIV_6
  *         @arg @ref LL_RCC_PLLP_DIV_8
  * @retval None
  */
/* *
  * @brief  Configure PLL used for 48Mhz domain clock
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLQ can be written only when PLL is disabled
  * @note This  can be selected for USB, RNG, SDMMC1
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLL_ConfigDomain_48M\n
  *         PLLCFGR      PLLM          LL_RCC_PLL_ConfigDomain_48M\n
  *         PLLCFGR      PLLN          LL_RCC_PLL_ConfigDomain_48M\n
  *         PLLCFGR      PLLQ          LL_RCC_PLL_ConfigDomain_48M
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLQ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLQ_DIV_2
  *         @arg @ref LL_RCC_PLLQ_DIV_3
  *         @arg @ref LL_RCC_PLLQ_DIV_4
  *         @arg @ref LL_RCC_PLLQ_DIV_5
  *         @arg @ref LL_RCC_PLLQ_DIV_6
  *         @arg @ref LL_RCC_PLLQ_DIV_7
  *         @arg @ref LL_RCC_PLLQ_DIV_8
  *         @arg @ref LL_RCC_PLLQ_DIV_9
  *         @arg @ref LL_RCC_PLLQ_DIV_10
  *         @arg @ref LL_RCC_PLLQ_DIV_11
  *         @arg @ref LL_RCC_PLLQ_DIV_12
  *         @arg @ref LL_RCC_PLLQ_DIV_13
  *         @arg @ref LL_RCC_PLLQ_DIV_14
  *         @arg @ref LL_RCC_PLLQ_DIV_15
  * @retval None
  */
/* DSI */
/* *
  * @brief  Get Main PLL multiplication factor for VCO
  * @rmtoll PLLCFGR      PLLN          LL_RCC_PLL_GetN
  * @retval Between 50 and 432
  */
/* *
  * @brief  Get Main PLL division factor for PLLP 
  * @rmtoll PLLCFGR      PLLP       LL_RCC_PLL_GetP
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLP_DIV_2
  *         @arg @ref LL_RCC_PLLP_DIV_4
  *         @arg @ref LL_RCC_PLLP_DIV_6
  *         @arg @ref LL_RCC_PLLP_DIV_8
  */
/* *
  * @brief  Get Main PLL division factor for PLLQ
  * @note used for PLL48MCLK selected for USB, RNG, SDMMC (48 MHz clock)
  * @rmtoll PLLCFGR      PLLQ          LL_RCC_PLL_GetQ
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLQ_DIV_2
  *         @arg @ref LL_RCC_PLLQ_DIV_3
  *         @arg @ref LL_RCC_PLLQ_DIV_4
  *         @arg @ref LL_RCC_PLLQ_DIV_5
  *         @arg @ref LL_RCC_PLLQ_DIV_6
  *         @arg @ref LL_RCC_PLLQ_DIV_7
  *         @arg @ref LL_RCC_PLLQ_DIV_8
  *         @arg @ref LL_RCC_PLLQ_DIV_9
  *         @arg @ref LL_RCC_PLLQ_DIV_10
  *         @arg @ref LL_RCC_PLLQ_DIV_11
  *         @arg @ref LL_RCC_PLLQ_DIV_12
  *         @arg @ref LL_RCC_PLLQ_DIV_13
  *         @arg @ref LL_RCC_PLLQ_DIV_14
  *         @arg @ref LL_RCC_PLLQ_DIV_15
  */
/* RCC_PLLCFGR_PLLR */
/* *
  * @brief  Get the oscillator used as PLL clock source.
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLL_GetMainSource
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  */
/* *
  * @brief  Get Division factor for the main PLL and other PLL
  * @rmtoll PLLCFGR      PLLM          LL_RCC_PLL_GetDivider
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  */
/* *
  * @brief  Configure Spread Spectrum used for PLL
  * @note These bits must be written before enabling PLL
  * @rmtoll SSCGR        MODPER        LL_RCC_PLL_ConfigSpreadSpectrum\n
  *         SSCGR        INCSTEP       LL_RCC_PLL_ConfigSpreadSpectrum\n
  *         SSCGR        SPREADSEL     LL_RCC_PLL_ConfigSpreadSpectrum
  * @param  Mod Between Min_Data=0 and Max_Data=8191
  * @param  Inc Between Min_Data=0 and Max_Data=32767
  * @param  Sel This parameter can be one of the following values:
  *         @arg @ref LL_RCC_SPREAD_SELECT_CENTER
  *         @arg @ref LL_RCC_SPREAD_SELECT_DOWN
  * @retval None
  */
/* *
  * @brief  Get Spread Spectrum Modulation Period for PLL
  * @rmtoll SSCGR         MODPER        LL_RCC_PLL_GetPeriodModulation
  * @retval Between Min_Data=0 and Max_Data=8191
  */
/* *
  * @brief  Get Spread Spectrum Incrementation Step for PLL
  * @note Must be written before enabling PLL
  * @rmtoll SSCGR         INCSTEP        LL_RCC_PLL_GetStepIncrementation
  * @retval Between Min_Data=0 and Max_Data=32767
  */
/* *
  * @brief  Get Spread Spectrum Selection for PLL
  * @note Must be written before enabling PLL
  * @rmtoll SSCGR         SPREADSEL        LL_RCC_PLL_GetSpreadSelection
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_SPREAD_SELECT_CENTER
  *         @arg @ref LL_RCC_SPREAD_SELECT_DOWN
  */
/* *
  * @brief  Enable Spread Spectrum for PLL.
  * @rmtoll SSCGR         SSCGEN         LL_RCC_PLL_SpreadSpectrum_Enable
  * @retval None
  */
/* *
  * @brief  Disable Spread Spectrum for PLL.
  * @rmtoll SSCGR         SSCGEN         LL_RCC_PLL_SpreadSpectrum_Disable
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_PLLI2S PLLI2S
  * @{
  */
/* *
  * @brief  Enable PLLI2S
  * @rmtoll CR           PLLI2SON     LL_RCC_PLLI2S_Enable
  * @retval None
  */
/* *
  * @brief  Disable PLLI2S
  * @rmtoll CR           PLLI2SON     LL_RCC_PLLI2S_Disable
  * @retval None
  */
/* *
  * @brief  Check if PLLI2S Ready
  * @rmtoll CR           PLLI2SRDY    LL_RCC_PLLI2S_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Configure PLLI2S used for SAI1 and SAI2 domain clock
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLQ can be written only when PLLI2S is disabled
  * @note This can be selected for SAI1 and SAI2
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLLI2S_ConfigDomain_SAI\n
  *         PLLCFGR      PLLM          LL_RCC_PLLI2S_ConfigDomain_SAI\n
  *         PLLI2SCFGR   PLLI2SN       LL_RCC_PLLI2S_ConfigDomain_SAI\n
  *         PLLI2SCFGR   PLLI2SQ       LL_RCC_PLLI2S_ConfigDomain_SAI\n
  *         DCKCFGR1      PLLI2SDIVQ    LL_RCC_PLLI2S_ConfigDomain_SAI
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLQ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_2
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_3
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_4
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_5
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_6
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_7
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_8
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_9
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_10
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_11
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_12
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_13
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_14
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_15
  * @param  PLLDIVQ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_1
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_2
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_3
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_4
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_5
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_6
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_7
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_8
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_9
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_10
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_11
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_12
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_13
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_14
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_15
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_16
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_17
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_18
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_19
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_20
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_21
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_22
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_23
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_24
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_25
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_26
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_27
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_28
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_29
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_30
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_31
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_32
  * @retval None
  */
/* SPDIFRX */
/* *
  * @brief  Configure PLLI2S used for I2S1 domain clock
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLR can be written only when PLLI2S is disabled
  * @note This  can be selected for I2S
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLLI2S_ConfigDomain_I2S\n
  *         PLLCFGR      PLLM          LL_RCC_PLLI2S_ConfigDomain_I2S\n
  *         PLLI2SCFGR   PLLI2SN       LL_RCC_PLLI2S_ConfigDomain_I2S\n
  *         PLLI2SCFGR   PLLI2SR       LL_RCC_PLLI2S_ConfigDomain_I2S
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLR This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SR_DIV_2
  *         @arg @ref LL_RCC_PLLI2SR_DIV_3
  *         @arg @ref LL_RCC_PLLI2SR_DIV_4
  *         @arg @ref LL_RCC_PLLI2SR_DIV_5
  *         @arg @ref LL_RCC_PLLI2SR_DIV_6
  *         @arg @ref LL_RCC_PLLI2SR_DIV_7
  * @retval None
  */
/* *
  * @brief  Get I2SPLL multiplication factor for VCO
  * @rmtoll PLLI2SCFGR  PLLI2SN      LL_RCC_PLLI2S_GetN
  * @retval Between 50 and 432
  */
/* *
  * @brief  Get I2SPLL division factor for PLLI2SQ
  * @rmtoll PLLI2SCFGR  PLLI2SQ      LL_RCC_PLLI2S_GetQ
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_2
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_3
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_4
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_5
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_6
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_7
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_8
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_9
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_10
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_11
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_12
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_13
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_14
  *         @arg @ref LL_RCC_PLLI2SQ_DIV_15
  */
/* *
  * @brief  Get I2SPLL division factor for PLLI2SR
  * @note used for PLLI2SCLK (I2S clock)
  * @rmtoll PLLI2SCFGR  PLLI2SR      LL_RCC_PLLI2S_GetR
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SR_DIV_2
  *         @arg @ref LL_RCC_PLLI2SR_DIV_3
  *         @arg @ref LL_RCC_PLLI2SR_DIV_4
  *         @arg @ref LL_RCC_PLLI2SR_DIV_5
  *         @arg @ref LL_RCC_PLLI2SR_DIV_6
  *         @arg @ref LL_RCC_PLLI2SR_DIV_7
  */
/* RCC_PLLI2SCFGR_PLLI2SP */
/* *
  * @brief  Get I2SPLL division factor for PLLI2SDIVQ
  * @note used PLLSAI1CLK, PLLSAI2CLK selected (SAI1 and SAI2 clock)
  * @rmtoll DCKCFGR1   PLLI2SDIVQ      LL_RCC_PLLI2S_GetDIVQ
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_1
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_2
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_3
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_4
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_5
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_6
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_7
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_8
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_9
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_10
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_11
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_12
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_13
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_14
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_15
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_16
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_17
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_18
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_19
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_20
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_21
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_22
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_23
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_24
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_25
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_26
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_27
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_28
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_29
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_30
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_31
  *         @arg @ref LL_RCC_PLLI2SDIVQ_DIV_32
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_PLLSAI PLLSAI
  * @{
  */
/* *
  * @brief  Enable PLLSAI
  * @rmtoll CR           PLLSAION     LL_RCC_PLLSAI_Enable
  * @retval None
  */
/* *
  * @brief  Disable PLLSAI
  * @rmtoll CR           PLLSAION     LL_RCC_PLLSAI_Disable
  * @retval None
  */
/* *
  * @brief  Check if PLLSAI Ready
  * @rmtoll CR           PLLSAIRDY    LL_RCC_PLLSAI_IsReady
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Configure PLLSAI used for SAI1 and SAI2 domain clock
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLQ can be written only when PLLSAI is disabled
  * @note This can be selected for SAI1 and SAI2
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLLSAI_ConfigDomain_SAI\n
  *         PLLCFGR      PLLM          LL_RCC_PLLSAI_ConfigDomain_SAI\n
  *         PLLSAICFGR   PLLSAIN       LL_RCC_PLLSAI_ConfigDomain_SAI\n
  *         PLLSAICFGR   PLLSAIQ       LL_RCC_PLLSAI_ConfigDomain_SAI\n
  *         DCKCFGR1     PLLSAIDIVQ    LL_RCC_PLLSAI_ConfigDomain_SAI
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLQ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_2
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_3
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_4
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_5
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_6
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_7
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_8
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_9
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_10
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_11
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_12
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_13
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_14
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_15
  * @param  PLLDIVQ This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_1
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_2
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_3
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_4
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_5
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_6
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_7
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_8
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_9
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_10
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_11
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_12
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_13
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_14
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_15
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_16
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_17
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_18
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_19
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_20
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_21
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_22
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_23
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_24
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_25
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_26
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_27
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_28
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_29
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_30
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_31
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_32
  * @retval None
  */
/* *
  * @brief Configure PLLSAI used for 48Mhz domain clock
  * @note PLL Source and PLLM Divider can be written only when PLL,
  *       PLLI2S and PLLSAI are disabled
  * @note PLLN/PLLP can be written only when PLLSAI is disabled
  * @note This  can be selected for USB, RNG, SDMMC1
  * @rmtoll PLLCFGR      PLLSRC        LL_RCC_PLLSAI_ConfigDomain_48M\n
  *         PLLCFGR      PLLM          LL_RCC_PLLSAI_ConfigDomain_48M\n
  *         PLLSAICFGR   PLLSAIN       LL_RCC_PLLSAI_ConfigDomain_48M\n
  *         PLLSAICFGR   PLLSAIP       LL_RCC_PLLSAI_ConfigDomain_48M
  * @param  Source This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSOURCE_HSI
  *         @arg @ref LL_RCC_PLLSOURCE_HSE
  * @param  PLLM This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLM_DIV_2
  *         @arg @ref LL_RCC_PLLM_DIV_3
  *         @arg @ref LL_RCC_PLLM_DIV_4
  *         @arg @ref LL_RCC_PLLM_DIV_5
  *         @arg @ref LL_RCC_PLLM_DIV_6
  *         @arg @ref LL_RCC_PLLM_DIV_7
  *         @arg @ref LL_RCC_PLLM_DIV_8
  *         @arg @ref LL_RCC_PLLM_DIV_9
  *         @arg @ref LL_RCC_PLLM_DIV_10
  *         @arg @ref LL_RCC_PLLM_DIV_11
  *         @arg @ref LL_RCC_PLLM_DIV_12
  *         @arg @ref LL_RCC_PLLM_DIV_13
  *         @arg @ref LL_RCC_PLLM_DIV_14
  *         @arg @ref LL_RCC_PLLM_DIV_15
  *         @arg @ref LL_RCC_PLLM_DIV_16
  *         @arg @ref LL_RCC_PLLM_DIV_17
  *         @arg @ref LL_RCC_PLLM_DIV_18
  *         @arg @ref LL_RCC_PLLM_DIV_19
  *         @arg @ref LL_RCC_PLLM_DIV_20
  *         @arg @ref LL_RCC_PLLM_DIV_21
  *         @arg @ref LL_RCC_PLLM_DIV_22
  *         @arg @ref LL_RCC_PLLM_DIV_23
  *         @arg @ref LL_RCC_PLLM_DIV_24
  *         @arg @ref LL_RCC_PLLM_DIV_25
  *         @arg @ref LL_RCC_PLLM_DIV_26
  *         @arg @ref LL_RCC_PLLM_DIV_27
  *         @arg @ref LL_RCC_PLLM_DIV_28
  *         @arg @ref LL_RCC_PLLM_DIV_29
  *         @arg @ref LL_RCC_PLLM_DIV_30
  *         @arg @ref LL_RCC_PLLM_DIV_31
  *         @arg @ref LL_RCC_PLLM_DIV_32
  *         @arg @ref LL_RCC_PLLM_DIV_33
  *         @arg @ref LL_RCC_PLLM_DIV_34
  *         @arg @ref LL_RCC_PLLM_DIV_35
  *         @arg @ref LL_RCC_PLLM_DIV_36
  *         @arg @ref LL_RCC_PLLM_DIV_37
  *         @arg @ref LL_RCC_PLLM_DIV_38
  *         @arg @ref LL_RCC_PLLM_DIV_39
  *         @arg @ref LL_RCC_PLLM_DIV_40
  *         @arg @ref LL_RCC_PLLM_DIV_41
  *         @arg @ref LL_RCC_PLLM_DIV_42
  *         @arg @ref LL_RCC_PLLM_DIV_43
  *         @arg @ref LL_RCC_PLLM_DIV_44
  *         @arg @ref LL_RCC_PLLM_DIV_45
  *         @arg @ref LL_RCC_PLLM_DIV_46
  *         @arg @ref LL_RCC_PLLM_DIV_47
  *         @arg @ref LL_RCC_PLLM_DIV_48
  *         @arg @ref LL_RCC_PLLM_DIV_49
  *         @arg @ref LL_RCC_PLLM_DIV_50
  *         @arg @ref LL_RCC_PLLM_DIV_51
  *         @arg @ref LL_RCC_PLLM_DIV_52
  *         @arg @ref LL_RCC_PLLM_DIV_53
  *         @arg @ref LL_RCC_PLLM_DIV_54
  *         @arg @ref LL_RCC_PLLM_DIV_55
  *         @arg @ref LL_RCC_PLLM_DIV_56
  *         @arg @ref LL_RCC_PLLM_DIV_57
  *         @arg @ref LL_RCC_PLLM_DIV_58
  *         @arg @ref LL_RCC_PLLM_DIV_59
  *         @arg @ref LL_RCC_PLLM_DIV_60
  *         @arg @ref LL_RCC_PLLM_DIV_61
  *         @arg @ref LL_RCC_PLLM_DIV_62
  *         @arg @ref LL_RCC_PLLM_DIV_63
  * @param  PLLN Between 50 and 432
  * @param  PLLP This parameter can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIP_DIV_2
  *         @arg @ref LL_RCC_PLLSAIP_DIV_4
  *         @arg @ref LL_RCC_PLLSAIP_DIV_6
  *         @arg @ref LL_RCC_PLLSAIP_DIV_8
  * @retval None
  */
/* LTDC */
/* *
  * @brief  Get SAIPLL multiplication factor for VCO
  * @rmtoll PLLSAICFGR  PLLSAIN      LL_RCC_PLLSAI_GetN
  * @retval Between 50 and 432
  */
/* *
  * @brief  Get SAIPLL division factor for PLLSAIQ
  * @rmtoll PLLSAICFGR  PLLSAIQ      LL_RCC_PLLSAI_GetQ
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_2
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_3
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_4
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_5
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_6
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_7
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_8
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_9
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_10
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_11
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_12
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_13
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_14
  *         @arg @ref LL_RCC_PLLSAIQ_DIV_15
  */
/* RCC_PLLSAICFGR_PLLSAIR */
/* *
  * @brief  Get SAIPLL division factor for PLLSAIP
  * @note used for PLL48MCLK (48M domain clock)
  * @rmtoll PLLSAICFGR  PLLSAIP      LL_RCC_PLLSAI_GetP
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIP_DIV_2
  *         @arg @ref LL_RCC_PLLSAIP_DIV_4
  *         @arg @ref LL_RCC_PLLSAIP_DIV_6
  *         @arg @ref LL_RCC_PLLSAIP_DIV_8
  */
/* *
  * @brief  Get SAIPLL division factor for PLLSAIDIVQ
  * @note used PLLSAI1CLK, PLLSAI2CLK selected (SAI1 and SAI2 clock)
  * @rmtoll DCKCFGR1   PLLSAIDIVQ      LL_RCC_PLLSAI_GetDIVQ
  * @retval Returned value can be one of the following values:
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_1
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_2
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_3
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_4
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_5
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_6
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_7
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_8
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_9
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_10
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_11
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_12
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_13
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_14
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_15
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_16
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_17
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_18
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_19
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_20
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_21
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_22
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_23
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_24
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_25
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_26
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_27
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_28
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_29
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_30
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_31
  *         @arg @ref LL_RCC_PLLSAIDIVQ_DIV_32
  */
/* RCC_DCKCFGR1_PLLSAIDIVR */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_FLAG_Management FLAG Management
  * @{
  */
/* *
  * @brief  Clear LSI ready interrupt flag
  * @rmtoll CIR         LSIRDYC       LL_RCC_ClearFlag_LSIRDY
  * @retval None
  */
/* *
  * @brief  Clear LSE ready interrupt flag
  * @rmtoll CIR         LSERDYC       LL_RCC_ClearFlag_LSERDY
  * @retval None
  */
/* *
  * @brief  Clear HSI ready interrupt flag
  * @rmtoll CIR         HSIRDYC       LL_RCC_ClearFlag_HSIRDY
  * @retval None
  */
/* *
  * @brief  Clear HSE ready interrupt flag
  * @rmtoll CIR         HSERDYC       LL_RCC_ClearFlag_HSERDY
  * @retval None
  */
/* *
  * @brief  Clear PLL ready interrupt flag
  * @rmtoll CIR         PLLRDYC       LL_RCC_ClearFlag_PLLRDY
  * @retval None
  */
/* *
  * @brief  Clear PLLI2S ready interrupt flag
  * @rmtoll CIR         PLLI2SRDYC   LL_RCC_ClearFlag_PLLI2SRDY
  * @retval None
  */
/* *
  * @brief  Clear PLLSAI ready interrupt flag
  * @rmtoll CIR         PLLSAIRDYC   LL_RCC_ClearFlag_PLLSAIRDY
  * @retval None
  */
/* *
  * @brief  Clear Clock security system interrupt flag
  * @rmtoll CIR         CSSC          LL_RCC_ClearFlag_HSECSS
  * @retval None
  */
/* *
  * @brief  Check if LSI ready interrupt occurred or not
  * @rmtoll CIR         LSIRDYF       LL_RCC_IsActiveFlag_LSIRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if LSE ready interrupt occurred or not
  * @rmtoll CIR         LSERDYF       LL_RCC_IsActiveFlag_LSERDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if HSI ready interrupt occurred or not
  * @rmtoll CIR         HSIRDYF       LL_RCC_IsActiveFlag_HSIRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if HSE ready interrupt occurred or not
  * @rmtoll CIR         HSERDYF       LL_RCC_IsActiveFlag_HSERDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if PLL ready interrupt occurred or not
  * @rmtoll CIR         PLLRDYF       LL_RCC_IsActiveFlag_PLLRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if PLLI2S ready interrupt occurred or not
  * @rmtoll CIR         PLLI2SRDYF   LL_RCC_IsActiveFlag_PLLI2SRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if PLLSAI ready interrupt occurred or not
  * @rmtoll CIR         PLLSAIRDYF   LL_RCC_IsActiveFlag_PLLSAIRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if Clock security system interrupt occurred or not
  * @rmtoll CIR         CSSF          LL_RCC_IsActiveFlag_HSECSS
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag Independent Watchdog reset is set or not.
  * @rmtoll CSR          IWDGRSTF      LL_RCC_IsActiveFlag_IWDGRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag Low Power reset is set or not.
  * @rmtoll CSR          LPWRRSTF      LL_RCC_IsActiveFlag_LPWRRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag Pin reset is set or not.
  * @rmtoll CSR          PINRSTF       LL_RCC_IsActiveFlag_PINRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag POR/PDR reset is set or not.
  * @rmtoll CSR          PORRSTF       LL_RCC_IsActiveFlag_PORRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag Software reset is set or not.
  * @rmtoll CSR          SFTRSTF       LL_RCC_IsActiveFlag_SFTRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag Window Watchdog reset is set or not.
  * @rmtoll CSR          WWDGRSTF      LL_RCC_IsActiveFlag_WWDGRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Check if RCC flag BOR reset is set or not.
  * @rmtoll CSR          BORRSTF       LL_RCC_IsActiveFlag_BORRST
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Set RMVF bit to clear the reset flags.
  * @rmtoll CSR          RMVF          LL_RCC_ClearResetFlags
  * @retval None
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_IT_Management IT Management
  * @{
  */
/* *
  * @brief  Enable LSI ready interrupt
  * @rmtoll CIR         LSIRDYIE      LL_RCC_EnableIT_LSIRDY
  * @retval None
  */
/* *
  * @brief  Enable LSE ready interrupt
  * @rmtoll CIR         LSERDYIE      LL_RCC_EnableIT_LSERDY
  * @retval None
  */
/* *
  * @brief  Enable HSI ready interrupt
  * @rmtoll CIR         HSIRDYIE      LL_RCC_EnableIT_HSIRDY
  * @retval None
  */
/* *
  * @brief  Enable HSE ready interrupt
  * @rmtoll CIR         HSERDYIE      LL_RCC_EnableIT_HSERDY
  * @retval None
  */
/* *
  * @brief  Enable PLL ready interrupt
  * @rmtoll CIR         PLLRDYIE      LL_RCC_EnableIT_PLLRDY
  * @retval None
  */
/* *
  * @brief  Enable PLLI2S ready interrupt
  * @rmtoll CIR         PLLI2SRDYIE  LL_RCC_EnableIT_PLLI2SRDY
  * @retval None
  */
/* *
  * @brief  Enable PLLSAI ready interrupt
  * @rmtoll CIR         PLLSAIRDYIE  LL_RCC_EnableIT_PLLSAIRDY
  * @retval None
  */
/* *
  * @brief  Disable LSI ready interrupt
  * @rmtoll CIR         LSIRDYIE      LL_RCC_DisableIT_LSIRDY
  * @retval None
  */
/* *
  * @brief  Disable LSE ready interrupt
  * @rmtoll CIR         LSERDYIE      LL_RCC_DisableIT_LSERDY
  * @retval None
  */
/* *
  * @brief  Disable HSI ready interrupt
  * @rmtoll CIR         HSIRDYIE      LL_RCC_DisableIT_HSIRDY
  * @retval None
  */
/* *
  * @brief  Disable HSE ready interrupt
  * @rmtoll CIR         HSERDYIE      LL_RCC_DisableIT_HSERDY
  * @retval None
  */
/* *
  * @brief  Disable PLL ready interrupt
  * @rmtoll CIR         PLLRDYIE      LL_RCC_DisableIT_PLLRDY
  * @retval None
  */
/* *
  * @brief  Disable PLLI2S ready interrupt
  * @rmtoll CIR         PLLI2SRDYIE  LL_RCC_DisableIT_PLLI2SRDY
  * @retval None
  */
/* *
  * @brief  Disable PLLSAI ready interrupt
  * @rmtoll CIR         PLLSAIRDYIE  LL_RCC_DisableIT_PLLSAIRDY
  * @retval None
  */
/* *
  * @brief  Checks if LSI ready interrupt source is enabled or disabled.
  * @rmtoll CIR         LSIRDYIE      LL_RCC_IsEnabledIT_LSIRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Checks if LSE ready interrupt source is enabled or disabled.
  * @rmtoll CIR         LSERDYIE      LL_RCC_IsEnabledIT_LSERDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Checks if HSI ready interrupt source is enabled or disabled.
  * @rmtoll CIR         HSIRDYIE      LL_RCC_IsEnabledIT_HSIRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Checks if HSE ready interrupt source is enabled or disabled.
  * @rmtoll CIR         HSERDYIE      LL_RCC_IsEnabledIT_HSERDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Checks if PLL ready interrupt source is enabled or disabled.
  * @rmtoll CIR         PLLRDYIE      LL_RCC_IsEnabledIT_PLLRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Checks if PLLI2S ready interrupt source is enabled or disabled.
  * @rmtoll CIR         PLLI2SRDYIE  LL_RCC_IsEnabledIT_PLLI2SRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @brief  Checks if PLLSAI ready interrupt source is enabled or disabled.
  * @rmtoll CIR         PLLSAIRDYIE  LL_RCC_IsEnabledIT_PLLSAIRDY
  * @retval State of bit (1 or 0).
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_Init De-initialization function
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_LL_EF_Get_Freq Get system and peripherals clocks frequency functions
  * @{
  */
/* CEC */
/* *
  * @brief  Return USBx clock frequency
  * @param  USBxSource This parameter can be one of the following values:
  *         @arg @ref LL_RCC_USB_CLKSOURCE
  * @retval USB clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn LL_RCC_GetUSBClockFreq(mut USBxSource: uint32_t)
 -> uint32_t {
    let mut usb_frequency: uint32_t = 0 as libc::c_uint;
    /* Check parameter */
    /* USBCLK clock frequency */
    match LL_RCC_GetUSBClockSource(USBxSource) {
        0 => {
            /* PLL clock used as USB clock source */
            if LL_RCC_PLL_IsReady() != 0 {
                usb_frequency = RCC_PLL_GetFreqDomain_48M()
            }
        }
        134217728 | _ => {
            /* PLLSAI clock used as USB clock source */
            if LL_RCC_PLLSAI_IsReady() != 0 {
                usb_frequency = RCC_PLLSAI_GetFreqDomain_48M()
            }
        }
    }
    return usb_frequency;
}
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_rcc.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   RCC LL module driver.
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
/* * @addtogroup RCC_LL
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @addtogroup RCC_LL_Private_Macros
  * @{
  */
/* I2C4 */
/* SDMMC2 */
/* DFSDM1_Channel0 */
/* CEC */
/* DSI */
/* LTDC */
/* SPDIFRX */
/* *
  * @}
  */
/* Private function prototypes -----------------------------------------------*/
/* * @defgroup RCC_LL_Private_Functions RCC Private functions
  * @{
  */
/* DFSDM1_Channel0 */
/* DSI */
/* LTDC */
/* SPDIFRX */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @addtogroup RCC_LL_Private_Functions
  * @{
  */
/* *
  * @brief  Return SYSTEM clock frequency
  * @retval SYSTEM clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetSystemClockFreq() -> uint32_t {
    let mut frequency: uint32_t = 0 as libc::c_uint;
    /* Get SYSCLK source -------------------------------------------------------*/
    match LL_RCC_GetSysClkSource() {
        0 => {
            /* HSI used as system clock  source */
            frequency = 16000000 as libc::c_uint
        }
        4 => {
            /* HSE used as system clock  source */
            frequency = 8000000 as libc::c_int as uint32_t
        }
        8 => {
            /* PLL used as system clock  source */
            frequency = RCC_PLL_GetFreqDomain_SYS()
        }
        _ => { frequency = 16000000 as libc::c_uint }
    }
    return frequency;
}
/* *
  * @brief  Return HCLK clock frequency
  * @param  SYSCLK_Frequency SYSCLK clock frequency
  * @retval HCLK clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetHCLKClockFreq(mut SYSCLK_Frequency: uint32_t)
 -> uint32_t {
    /* HCLK clock frequency */
    return SYSCLK_Frequency >>
               AHBPrescTable[((LL_RCC_GetAHBPrescaler() &
                                   (0xf as libc::c_uint) << 4 as libc::c_uint)
                                  >> 4 as libc::c_uint) as usize] as
                   libc::c_int;
}
/* *
  * @brief  Return PCLK1 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK1 clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetPCLK1ClockFreq(mut HCLK_Frequency: uint32_t)
 -> uint32_t {
    /* PCLK1 clock frequency */
    return HCLK_Frequency >>
               APBPrescTable[(LL_RCC_GetAPB1Prescaler() >> 10 as libc::c_uint)
                                 as usize] as libc::c_int;
}
/* *
  * @brief  Return PCLK2 clock frequency
  * @param  HCLK_Frequency HCLK clock frequency
  * @retval PCLK2 clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetPCLK2ClockFreq(mut HCLK_Frequency: uint32_t)
 -> uint32_t {
    /* PCLK2 clock frequency */
    return HCLK_Frequency >>
               APBPrescTable[(LL_RCC_GetAPB2Prescaler() >> 13 as libc::c_uint)
                                 as usize] as libc::c_int;
}
/* *
  * @brief  Return PLL clock frequency used for system domain
  * @retval PLL clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLL_GetFreqDomain_SYS() -> uint32_t {
    let mut pllinputfreq: uint32_t = 0 as libc::c_uint;
    let mut pllsource: uint32_t = 0 as libc::c_uint;
    /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
     SYSCLK = PLL_VCO / PLLP
  */
    pllsource = LL_RCC_PLL_GetMainSource();
    match pllsource {
        0 => {
            /* HSI used as PLL clock source */
            pllinputfreq = 16000000 as libc::c_uint
        }
        4194304 => {
            /* HSE used as PLL clock source */
            pllinputfreq = 8000000 as libc::c_int as uint32_t
        }
        _ => { pllinputfreq = 16000000 as libc::c_uint }
    }
    return pllinputfreq.wrapping_div(LL_RCC_PLL_GetDivider()).wrapping_mul(LL_RCC_PLL_GetN()).wrapping_div((LL_RCC_PLL_GetP()
                                                                                                                >>
                                                                                                                16
                                                                                                                    as
                                                                                                                    libc::c_uint).wrapping_add(1
                                                                                                                                                   as
                                                                                                                                                   libc::c_uint).wrapping_mul(2
                                                                                                                                                                                  as
                                                                                                                                                                                  libc::c_uint));
}
/* *
  * @brief  Return PLL clock frequency used for 48 MHz domain
  * @retval PLL clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLL_GetFreqDomain_48M() -> uint32_t {
    let mut pllinputfreq: uint32_t = 0 as libc::c_uint;
    let mut pllsource: uint32_t = 0 as libc::c_uint;
    /* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM ) * PLLN
     48M Domain clock = PLL_VCO / PLLQ
  */
    pllsource = LL_RCC_PLL_GetMainSource();
    match pllsource {
        0 => {
            /* HSI used as PLL clock source */
            pllinputfreq = 16000000 as libc::c_uint
        }
        4194304 => {
            /* HSE used as PLL clock source */
            pllinputfreq = 8000000 as libc::c_int as uint32_t
        }
        _ => { pllinputfreq = 16000000 as libc::c_uint }
    }
    return pllinputfreq.wrapping_div(LL_RCC_PLL_GetDivider()).wrapping_mul(LL_RCC_PLL_GetN()).wrapping_div(LL_RCC_PLL_GetQ()
                                                                                                               >>
                                                                                                               24
                                                                                                                   as
                                                                                                                   libc::c_uint);
}
/* DSI */
/* DSI */
/* *
  * @brief  Return PLLSAI clock frequency used for SAI1 and SAI2 domains
  * @retval PLLSAI clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLSAI_GetFreqDomain_SAI() -> uint32_t {
    let mut pllinputfreq: uint32_t = 0 as libc::c_uint;
    let mut pllsource: uint32_t = 0 as libc::c_uint;
    /* PLLSAI_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLSAIN
     SAI1 and SAI2 domains clock  = (PLLSAI_VCO / PLLSAIQ) / PLLSAIDIVQ
  */
    pllsource = LL_RCC_PLL_GetMainSource();
    match pllsource {
        0 => {
            /* HSI used as PLLSAI clock source */
            pllinputfreq = 16000000 as libc::c_uint
        }
        4194304 => {
            /* HSE used as PLLSAI clock source */
            pllinputfreq = 8000000 as libc::c_int as uint32_t
        }
        _ => { pllinputfreq = 16000000 as libc::c_uint }
    }
    return pllinputfreq.wrapping_div(LL_RCC_PLL_GetDivider()).wrapping_mul(LL_RCC_PLLSAI_GetN()).wrapping_div((LL_RCC_PLLSAI_GetQ()
                                                                                                                   >>
                                                                                                                   24
                                                                                                                       as
                                                                                                                       libc::c_uint).wrapping_mul((LL_RCC_PLLSAI_GetDIVQ()
                                                                                                                                                       >>
                                                                                                                                                       8
                                                                                                                                                           as
                                                                                                                                                           libc::c_uint).wrapping_add(1
                                                                                                                                                                                          as
                                                                                                                                                                                          libc::c_uint)));
}
/* *
  * @brief  Return PLLSAI clock frequency used for 48Mhz domain
  * @retval PLLSAI clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLSAI_GetFreqDomain_48M() -> uint32_t {
    let mut pllinputfreq: uint32_t = 0 as libc::c_uint;
    let mut pllsource: uint32_t = 0 as libc::c_uint;
    /* PLLSAI_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLSAIN
     48M Domain clock  = PLLSAI_VCO / PLLSAIP
  */
    pllsource = LL_RCC_PLL_GetMainSource();
    match pllsource {
        0 => {
            /* HSI used as PLLSAI clock source */
            pllinputfreq = 16000000 as libc::c_uint
        }
        4194304 => {
            /* HSE used as PLLSAI clock source */
            pllinputfreq = 8000000 as libc::c_int as uint32_t
        }
        _ => { pllinputfreq = 16000000 as libc::c_uint }
    }
    return pllinputfreq.wrapping_div(LL_RCC_PLL_GetDivider()).wrapping_mul(LL_RCC_PLLSAI_GetN()).wrapping_div((LL_RCC_PLLSAI_GetP()
                                                                                                                   >>
                                                                                                                   16
                                                                                                                       as
                                                                                                                       libc::c_uint).wrapping_add(1
                                                                                                                                                      as
                                                                                                                                                      libc::c_uint).wrapping_mul(2
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint));
}
/* LTDC */
/* *
  * @brief  Return PLLI2S clock frequency used for SAI1 and SAI2 domains
  * @retval PLLI2S clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLI2S_GetFreqDomain_SAI() -> uint32_t {
    let mut pllinputfreq: uint32_t = 0 as libc::c_uint;
    let mut pllsource: uint32_t = 0 as libc::c_uint;
    /* PLLI2S_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLI2SN
     SAI1 and SAI2 domains clock  = (PLLI2S_VCO / PLLI2SQ) / PLLI2SDIVQ
  */
    pllsource = LL_RCC_PLL_GetMainSource();
    match pllsource {
        0 => {
            /* HSI used as PLLI2S clock source */
            pllinputfreq = 16000000 as libc::c_uint
        }
        4194304 => {
            /* HSE used as PLLI2S clock source */
            pllinputfreq = 8000000 as libc::c_int as uint32_t
        }
        _ => { pllinputfreq = 16000000 as libc::c_uint }
    }
    return pllinputfreq.wrapping_div(LL_RCC_PLL_GetDivider()).wrapping_mul(LL_RCC_PLLI2S_GetN()).wrapping_div((LL_RCC_PLLI2S_GetQ()
                                                                                                                   >>
                                                                                                                   24
                                                                                                                       as
                                                                                                                       libc::c_uint).wrapping_mul((LL_RCC_PLLI2S_GetDIVQ()
                                                                                                                                                       >>
                                                                                                                                                       0
                                                                                                                                                           as
                                                                                                                                                           libc::c_uint).wrapping_add(1
                                                                                                                                                                                          as
                                                                                                                                                                                          libc::c_uint)));
}
/* LTDC */
/* SPDIFRX */
/* *
  * @brief  Return PLLI2S clock frequency used for I2S domain
  * @retval PLLI2S clock frequency (in Hz)
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLI2S_GetFreqDomain_I2S() -> uint32_t {
    let mut pllinputfreq: uint32_t = 0 as libc::c_uint;
    let mut pllsource: uint32_t = 0 as libc::c_uint;
    /* PLLI2S_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLI2SN
     I2S Domain clock  = PLLI2S_VCO / PLLI2SR
  */
    pllsource = LL_RCC_PLL_GetMainSource();
    match pllsource {
        4194304 => {
            /* HSE used as PLLI2S clock source */
            pllinputfreq = 8000000 as libc::c_int as uint32_t
        }
        0 | _ => {
            /* HSI used as PLLI2S clock source */
            pllinputfreq = 16000000 as libc::c_uint
        }
    }
    return pllinputfreq.wrapping_div(LL_RCC_PLL_GetDivider()).wrapping_mul(LL_RCC_PLLI2S_GetN()).wrapping_div(LL_RCC_PLLI2S_GetR()
                                                                                                                  >>
                                                                                                                  28
                                                                                                                      as
                                                                                                                      libc::c_uint);
}
/* *********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* USE_FULL_LL_DRIVER */
/* *
  * @}
  */
/* defined(RCC) */
/* *
  * @}
  */
/* *
  * @}
  */
