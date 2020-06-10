use ::libc;
extern "C" {
    #[no_mangle]
    static mut hse_value: uint32_t;
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
  * @brief Reset and Clock Control
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RCC_TypeDef {
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub CIR: uint32_t,
    pub APB2RSTR: uint32_t,
    pub APB1RSTR: uint32_t,
    pub AHBENR: uint32_t,
    pub APB2ENR: uint32_t,
    pub APB1ENR: uint32_t,
    pub BDCR: uint32_t,
    pub CSR: uint32_t,
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
/* *
  * @}
  */
/* * @defgroup RCC_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Private_Variables
  * @{
  */
static mut APBAHBPrescTable: [uint8_t; 16] =
    [0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     0 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     1 as libc::c_int as uint8_t, 2 as libc::c_int as uint8_t,
     3 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 7 as libc::c_int as uint8_t,
     8 as libc::c_int as uint8_t, 9 as libc::c_int as uint8_t];
static mut ADCPrescTable: [uint8_t; 4] =
    [2 as libc::c_int as uint8_t, 4 as libc::c_int as uint8_t,
     6 as libc::c_int as uint8_t, 8 as libc::c_int as uint8_t];
/* *
  * @}
  */
/* * @defgroup RCC_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Private_Functions
  * @{
  */
/* *
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_DeInit() {
    /* Set HSION bit */
    let ref mut fresh0 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    let ref mut fresh1 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xf8ff0000 as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* STM32F10X_CL */
    /* Reset HSEON, CSSON and PLLON bits */
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfef6ffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset HSEBYP bit */
    let ref mut fresh3 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffbffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    let ref mut fresh4 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xff80ffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Disable all interrupts and clear pending bits  */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CIR as
                                    *mut uint32_t,
                                0x9f0000 as libc::c_int as uint32_t);
    /* STM32F10X_CL */
}
/* *
  * @brief  Configures the External High Speed oscillator (HSE).
  * @note   HSE can not be stopped if it is used directly or through the PLL as system clock.
  * @param  RCC_HSE: specifies the new state of the HSE.
  *   This parameter can be one of the following values:
  *     @arg RCC_HSE_OFF: HSE oscillator OFF
  *     @arg RCC_HSE_ON: HSE oscillator ON
  *     @arg RCC_HSE_Bypass: HSE oscillator bypassed with external clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HSEConfig(mut RCC_HSE: uint32_t) {
    /* Check the parameters */
    /* Reset HSEON and HSEBYP bits before configuring the HSE ------------------*/
  /* Reset HSEON bit */
    let ref mut fresh5 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffeffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Reset HSEBYP bit */
    let ref mut fresh6 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xfffbffff as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Configure HSE (RCC_HSE_OFF is already covered by the code section above) */
    match RCC_HSE {
        65536 => {
            /* Set HSEON bit */
            let ref mut fresh7 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh7,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x10000 as libc::c_int as
                                                 uint32_t) as uint32_t as
                                            uint32_t)
        }
        262144 => {
            /* Set HSEBYP and HSEON bits */
            let ref mut fresh8 =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh8,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             (0x40000 as libc::c_int as
                                                  uint32_t |
                                                  0x10000 as libc::c_int as
                                                      uint32_t)) as uint32_t
                                            as uint32_t)
        }
        _ => { }
    };
}
/* *
  * @brief  Waits for HSE start-up.
  * @param  None
  * @retval An ErrorStatus enumuration value:
  * - SUCCESS: HSE oscillator is stable and ready to use
  * - ERROR: HSE oscillator not yet ready
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_WaitForHSEStartUp() -> ErrorStatus {
    let mut StartUpCounter: uint32_t = 0 as libc::c_int as uint32_t;
    let mut status: ErrorStatus = ERROR;
    let mut HSEStatus: FlagStatus = RESET;
    loop 
         /* Wait till HSE is ready and if Time out is reached exit */
         {
        HSEStatus = RCC_GetFlagStatus(0x31 as libc::c_int as uint8_t);
        ::core::ptr::write_volatile(&mut StartUpCounter as *mut uint32_t,
                                    ::core::ptr::read_volatile::<uint32_t>(&StartUpCounter
                                                                               as
                                                                               *const uint32_t).wrapping_add(1));
        if !(StartUpCounter !=
                 0x500 as libc::c_int as uint16_t as libc::c_uint &&
                 HSEStatus as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) {
            break ;
        }
    }
    if RCC_GetFlagStatus(0x31 as libc::c_int as uint8_t) as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        status = SUCCESS
    } else { status = ERROR }
    return status;
}
/* *
  * @brief  Adjusts the Internal High Speed oscillator (HSI) calibration value.
  * @param  HSICalibrationValue: specifies the calibration trimming value.
  *   This parameter must be a number between 0 and 0x1F.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_AdjustHSICalibrationValue(mut HSICalibrationValue:
                                                           uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    /* Clear HSITRIM[4:0] bits */
    tmpreg &= 0xffffff07 as libc::c_uint;
    /* Set the HSITRIM[4:0] bits according to HSICalibrationValue value */
    tmpreg |= (HSICalibrationValue as uint32_t) << 3 as libc::c_int;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the Internal High Speed oscillator (HSI).
  * @note   HSI can not be stopped if it is used directly or through the PLL as system clock.
  * @param  NewState: new state of the HSI. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HSICmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
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
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0
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
  * @brief  Configures the PLL clock source and multiplication factor.
  * @note   This function must be used only when the PLL is disabled.
  * @param  RCC_PLLSource: specifies the PLL entry clock source.
  *   For @b STM32_Connectivity_line_devices or @b STM32_Value_line_devices, 
  *   this parameter can be one of the following values:
  *     @arg RCC_PLLSource_HSI_Div2: HSI oscillator clock divided by 2 selected as PLL clock entry
  *     @arg RCC_PLLSource_PREDIV1: PREDIV1 clock selected as PLL clock entry
  *   For @b other_STM32_devices, this parameter can be one of the following values:
  *     @arg RCC_PLLSource_HSI_Div2: HSI oscillator clock divided by 2 selected as PLL clock entry
  *     @arg RCC_PLLSource_HSE_Div1: HSE oscillator clock selected as PLL clock entry
  *     @arg RCC_PLLSource_HSE_Div2: HSE oscillator clock divided by 2 selected as PLL clock entry 
  * @param  RCC_PLLMul: specifies the PLL multiplication factor.
  *   For @b STM32_Connectivity_line_devices, this parameter can be RCC_PLLMul_x where x:{[4,9], 6_5}
  *   For @b other_STM32_devices, this parameter can be RCC_PLLMul_x where x:[2,16]  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLConfig(mut RCC_PLLSource: uint32_t,
                                       mut RCC_PLLMul: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear PLLSRC, PLLXTPRE and PLLMUL[3:0] bits */
    tmpreg &= 0xffc0ffff as libc::c_uint;
    /* Set the PLL configuration bits */
    tmpreg |= RCC_PLLSource | RCC_PLLMul;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the PLL.
  * @note   The PLL can not be disabled if it is used as system clock.
  * @param  NewState: new state of the PLL. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
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
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x18
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
/* STM32F10X_CL */
/* *
  * @brief  Configures the system clock (SYSCLK).
  * @param  RCC_SYSCLKSource: specifies the clock source used as system clock.
  *   This parameter can be one of the following values:
  *     @arg RCC_SYSCLKSource_HSI: HSI selected as system clock
  *     @arg RCC_SYSCLKSource_HSE: HSE selected as system clock
  *     @arg RCC_SYSCLKSource_PLLCLK: PLL selected as system clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_SYSCLKConfig(mut RCC_SYSCLKSource: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear SW[1:0] bits */
    tmpreg &= 0xfffffffc as libc::c_uint;
    /* Set SW[1:0] bits according to RCC_SYSCLKSource value */
    tmpreg |= RCC_SYSCLKSource;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Returns the clock source used as system clock.
  * @param  None
  * @retval The clock source used as system clock. The returned value can
  *   be one of the following:
  *     - 0x00: HSI used as system clock
  *     - 0x04: HSE used as system clock
  *     - 0x08: PLL used as system clock
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetSYSCLKSource() -> uint8_t {
    return ((*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR &
                0xc as libc::c_int as uint32_t) as uint8_t;
}
/* *
  * @brief  Configures the AHB clock (HCLK).
  * @param  RCC_SYSCLK: defines the AHB clock divider. This clock is derived from 
  *   the system clock (SYSCLK).
  *   This parameter can be one of the following values:
  *     @arg RCC_SYSCLK_Div1: AHB clock = SYSCLK
  *     @arg RCC_SYSCLK_Div2: AHB clock = SYSCLK/2
  *     @arg RCC_SYSCLK_Div4: AHB clock = SYSCLK/4
  *     @arg RCC_SYSCLK_Div8: AHB clock = SYSCLK/8
  *     @arg RCC_SYSCLK_Div16: AHB clock = SYSCLK/16
  *     @arg RCC_SYSCLK_Div64: AHB clock = SYSCLK/64
  *     @arg RCC_SYSCLK_Div128: AHB clock = SYSCLK/128
  *     @arg RCC_SYSCLK_Div256: AHB clock = SYSCLK/256
  *     @arg RCC_SYSCLK_Div512: AHB clock = SYSCLK/512
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HCLKConfig(mut RCC_SYSCLK: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear HPRE[3:0] bits */
    tmpreg &= 0xffffff0f as libc::c_uint;
    /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
    tmpreg |= RCC_SYSCLK;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the Low Speed APB clock (PCLK1).
  * @param  RCC_HCLK: defines the APB1 clock divider. This clock is derived from 
  *   the AHB clock (HCLK).
  *   This parameter can be one of the following values:
  *     @arg RCC_HCLK_Div1: APB1 clock = HCLK
  *     @arg RCC_HCLK_Div2: APB1 clock = HCLK/2
  *     @arg RCC_HCLK_Div4: APB1 clock = HCLK/4
  *     @arg RCC_HCLK_Div8: APB1 clock = HCLK/8
  *     @arg RCC_HCLK_Div16: APB1 clock = HCLK/16
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PCLK1Config(mut RCC_HCLK: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear PPRE1[2:0] bits */
    tmpreg &= 0xfffff8ff as libc::c_uint;
    /* Set PPRE1[2:0] bits according to RCC_HCLK value */
    tmpreg |= RCC_HCLK;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the High Speed APB clock (PCLK2).
  * @param  RCC_HCLK: defines the APB2 clock divider. This clock is derived from 
  *   the AHB clock (HCLK).
  *   This parameter can be one of the following values:
  *     @arg RCC_HCLK_Div1: APB2 clock = HCLK
  *     @arg RCC_HCLK_Div2: APB2 clock = HCLK/2
  *     @arg RCC_HCLK_Div4: APB2 clock = HCLK/4
  *     @arg RCC_HCLK_Div8: APB2 clock = HCLK/8
  *     @arg RCC_HCLK_Div16: APB2 clock = HCLK/16
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PCLK2Config(mut RCC_HCLK: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear PPRE2[2:0] bits */
    tmpreg &= 0xffffc7ff as libc::c_uint;
    /* Set PPRE2[2:0] bits according to RCC_HCLK value */
    tmpreg |= RCC_HCLK << 3 as libc::c_int;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the specified RCC interrupts.
  * @param  RCC_IT: specifies the RCC interrupt sources to be enabled or disabled.
  * 
  *   For @b STM32_Connectivity_line_devices, this parameter can be any combination
  *   of the following values        
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *     @arg RCC_IT_PLL2RDY: PLL2 ready interrupt
  *     @arg RCC_IT_PLL3RDY: PLL3 ready interrupt
  * 
  *   For @b other_STM32_devices, this parameter can be any combination of the 
  *   following values        
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *       
  * @param  NewState: new state of the specified RCC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ITConfig(mut RCC_IT: uint8_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Perform Byte access to RCC_CIR bits to enable the selected interrupts */
        let ref mut fresh9 =
            *(0x40021009 as libc::c_int as uint32_t as *mut uint8_t);
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint8_t>(fresh9
                                                                               as
                                                                               *const uint8_t)
                                         as libc::c_int |
                                         RCC_IT as libc::c_int) as uint8_t as
                                        uint8_t)
    } else {
        /* Perform Byte access to RCC_CIR bits to disable the selected interrupts */
        let ref mut fresh10 =
            *(0x40021009 as libc::c_int as uint32_t as *mut uint8_t);
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint8_t>(fresh10
                                                                               as
                                                                               *const uint8_t)
                                         as libc::c_int &
                                         !(RCC_IT as libc::c_int) as uint8_t
                                             as libc::c_int) as uint8_t as
                                        uint8_t)
    };
}
/* *
  * @brief  Configures the USB clock (USBCLK).
  * @param  RCC_USBCLKSource: specifies the USB clock source. This clock is 
  *   derived from the PLL output.
  *   This parameter can be one of the following values:
  *     @arg RCC_USBCLKSource_PLLCLK_1Div5: PLL clock divided by 1,5 selected as USB 
  *                                     clock source
  *     @arg RCC_USBCLKSource_PLLCLK_Div1: PLL clock selected as USB clock source
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_USBCLKConfig(mut RCC_USBCLKSource: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
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
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x16
                                                                                                                                                                                                                                                     as
                                                                                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4
                                                                                                                                                                                                                                                         as
                                                                                                                                                                                                                                                         libc::c_int)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, RCC_USBCLKSource);
}
/* STM32F10X_CL */
/* *
  * @brief  Configures the ADC clock (ADCCLK).
  * @param  RCC_PCLK2: defines the ADC clock divider. This clock is derived from 
  *   the APB2 clock (PCLK2).
  *   This parameter can be one of the following values:
  *     @arg RCC_PCLK2_Div2: ADC clock = PCLK2/2
  *     @arg RCC_PCLK2_Div4: ADC clock = PCLK2/4
  *     @arg RCC_PCLK2_Div6: ADC clock = PCLK2/6
  *     @arg RCC_PCLK2_Div8: ADC clock = PCLK2/8
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ADCCLKConfig(mut RCC_PCLK2: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear ADCPRE[1:0] bits */
    tmpreg &= 0xffff3fff as libc::c_uint;
    /* Set ADCPRE[1:0] bits according to RCC_PCLK2 value */
    tmpreg |= RCC_PCLK2;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x20000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000
                                                                                                       as
                                                                                                       libc::c_int
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* STM32F10X_CL */
/* *
  * @brief  Configures the External Low Speed oscillator (LSE).
  * @param  RCC_LSE: specifies the new state of the LSE.
  *   This parameter can be one of the following values:
  *     @arg RCC_LSE_OFF: LSE oscillator OFF
  *     @arg RCC_LSE_ON: LSE oscillator ON
  *     @arg RCC_LSE_Bypass: LSE oscillator bypassed with external clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_LSEConfig(mut RCC_LSE: uint8_t) {
    /* Check the parameters */
    /* Reset LSEON and LSEBYP bits before configuring the LSE ------------------*/
  /* Reset LSEON bit */
    ::core::ptr::write_volatile((0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x20
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint))
                                    as *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    /* Reset LSEBYP bit */
    ::core::ptr::write_volatile((0x40000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x20
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint))
                                    as *mut uint8_t,
                                0 as libc::c_int as uint8_t);
    /* Configure LSE (RCC_LSE_OFF is already covered by the code section above) */
    match RCC_LSE as libc::c_int {
        1 => {
            /* Set LSEON bit */
            ::core::ptr::write_volatile((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add((0x40000000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         uint32_t).wrapping_add(0x20000
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0x1000
                                                                                                                                   as
                                                                                                                                   libc::c_int
                                                                                                                                   as
                                                                                                                                   libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                                  as
                                                                                                                                                                  libc::c_int
                                                                                                                                                                  as
                                                                                                                                                                  uint32_t).wrapping_add(0x20
                                                                                                                                                                                             as
                                                                                                                                                                                             libc::c_int
                                                                                                                                                                                             as
                                                                                                                                                                                             libc::c_uint))
                                            as *mut uint8_t,
                                        0x1 as libc::c_int as uint8_t)
        }
        4 => {
            /* Set LSEBYP and LSEON bits */
            ::core::ptr::write_volatile((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add((0x40000000
                                                                         as
                                                                         libc::c_int
                                                                         as
                                                                         uint32_t).wrapping_add(0x20000
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint).wrapping_add(0x1000
                                                                                                                                   as
                                                                                                                                   libc::c_int
                                                                                                                                   as
                                                                                                                                   libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                                  as
                                                                                                                                                                  libc::c_int
                                                                                                                                                                  as
                                                                                                                                                                  uint32_t).wrapping_add(0x20
                                                                                                                                                                                             as
                                                                                                                                                                                             libc::c_int
                                                                                                                                                                                             as
                                                                                                                                                                                             libc::c_uint))
                                            as *mut uint8_t,
                                        (0x4 as libc::c_int as uint8_t as
                                             libc::c_int |
                                             0x1 as libc::c_int as uint8_t as
                                                 libc::c_int) as uint8_t)
        }
        _ => { }
    };
}
/* *
  * @brief  Enables or disables the Internal Low Speed oscillator (LSI).
  * @note   LSI can not be disabled if the IWDG is running.
  * @param  NewState: new state of the LSI. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_LSICmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x24
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_int
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0
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
  * @brief  Configures the RTC clock (RTCCLK).
  * @note   Once the RTC clock is selected it can't be changed unless the Backup domain is reset.
  * @param  RCC_RTCCLKSource: specifies the RTC clock source.
  *   This parameter can be one of the following values:
  *     @arg RCC_RTCCLKSource_LSE: LSE selected as RTC clock
  *     @arg RCC_RTCCLKSource_LSI: LSI selected as RTC clock
  *     @arg RCC_RTCCLKSource_HSE_Div128: HSE clock divided by 128 selected as RTC clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_RTCCLKConfig(mut RCC_RTCCLKSource: uint32_t) {
    /* Check the parameters */
    /* Select the RTC clock source */
    let ref mut fresh11 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_RTCCLKSource) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the RTC clock.
  * @note   This function must be used only after the RTC clock was selected using the RCC_RTCCLKConfig function.
  * @param  NewState: new state of the RTC clock. This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_RTCCLKCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x20
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_int
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0xf
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
  * @brief  Returns the frequencies of different on chip clocks.
  * @param  RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold
  *         the clocks frequencies.
  * @note   The result of this function could be not correct when using 
  *         fractional value for HSE crystal.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetClocksFreq(mut RCC_Clocks:
                                               *mut RCC_ClocksTypeDef) {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllmull: uint32_t = 0 as libc::c_int as uint32_t;
    let mut pllsource: uint32_t = 0 as libc::c_int as uint32_t;
    let mut presc: uint32_t = 0 as libc::c_int as uint32_t;
    /* STM32F10X_CL */
    /* Get SYSCLK source -------------------------------------------------------*/
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0xc as libc::c_int as uint32_t;
    match tmp {
        0 => {
            /* HSI used as system clock */
            (*RCC_Clocks).SYSCLK_Frequency =
                8000000 as libc::c_int as uint32_t
        }
        4 => {
            /* HSE used as system clock */
            (*RCC_Clocks).SYSCLK_Frequency = hse_value
        }
        8 => {
            /* PLL used as system clock */
            /* Get PLL clock source and multiplication factor ----------------------*/
            pllmull =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR &
                    0x3c0000 as libc::c_int as uint32_t;
            pllsource =
                (*((0x40000000 as libc::c_int as
                        uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                   libc::c_uint).wrapping_add(0x1000
                                                                                  as
                                                                                  libc::c_int
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR &
                    0x10000 as libc::c_int as uint32_t;
            pllmull =
                (pllmull >>
                     18 as
                         libc::c_int).wrapping_add(2 as libc::c_int as
                                                       libc::c_uint);
            if pllsource == 0 as libc::c_int as libc::c_uint {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                (*RCC_Clocks).SYSCLK_Frequency =
                    (8000000 as libc::c_int as uint32_t >>
                         1 as libc::c_int).wrapping_mul(pllmull)
            } else if (*((0x40000000 as libc::c_int as
                              uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                         libc::c_uint).wrapping_add(0x1000
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        libc::c_uint)
                             as *mut RCC_TypeDef)).CFGR &
                          0x20000 as libc::c_int as uint32_t !=
                          RESET as libc::c_int as uint32_t {
                /* HSE selected as PLL clock entry */
                /* HSE oscillator clock divided by 2 */
                (*RCC_Clocks).SYSCLK_Frequency =
                    (hse_value >> 1 as libc::c_int).wrapping_mul(pllmull)
            } else {
                (*RCC_Clocks).SYSCLK_Frequency =
                    hse_value.wrapping_mul(pllmull)
            }
        }
        _ => {
            (*RCC_Clocks).SYSCLK_Frequency =
                8000000 as libc::c_int as uint32_t
        }
    }
    /* Compute HCLK, PCLK1, PCLK2 and ADCCLK clocks frequencies ----------------*/
  /* Get HCLK prescaler */
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0xf0 as libc::c_int as uint32_t;
    tmp = tmp >> 4 as libc::c_int;
    presc = APBAHBPrescTable[tmp as usize] as uint32_t;
    /* HCLK clock frequency */
    (*RCC_Clocks).HCLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency >> presc;
    /* Get PCLK1 prescaler */
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0x700 as libc::c_int as uint32_t;
    tmp = tmp >> 8 as libc::c_int;
    presc = APBAHBPrescTable[tmp as usize] as uint32_t;
    /* PCLK1 clock frequency */
    (*RCC_Clocks).PCLK1_Frequency = (*RCC_Clocks).HCLK_Frequency >> presc;
    /* Get PCLK2 prescaler */
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0x3800 as libc::c_int as uint32_t;
    tmp = tmp >> 11 as libc::c_int;
    presc = APBAHBPrescTable[tmp as usize] as uint32_t;
    /* PCLK2 clock frequency */
    (*RCC_Clocks).PCLK2_Frequency = (*RCC_Clocks).HCLK_Frequency >> presc;
    /* Get ADCCLK prescaler */
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0xc000 as libc::c_int as uint32_t;
    tmp = tmp >> 14 as libc::c_int;
    presc = ADCPrescTable[tmp as usize] as uint32_t;
    /* ADCCLK clock frequency */
    (*RCC_Clocks).ADCCLK_Frequency =
        (*RCC_Clocks).PCLK2_Frequency.wrapping_div(presc);
}
/* *
  * @brief  Enables or disables the AHB peripheral clock.
  * @param  RCC_AHBPeriph: specifies the AHB peripheral to gates its clock.
  *   
  *   For @b STM32_Connectivity_line_devices, this parameter can be any combination
  *   of the following values:        
  *     @arg RCC_AHBPeriph_DMA1
  *     @arg RCC_AHBPeriph_DMA2
  *     @arg RCC_AHBPeriph_SRAM
  *     @arg RCC_AHBPeriph_FLITF
  *     @arg RCC_AHBPeriph_CRC
  *     @arg RCC_AHBPeriph_OTG_FS    
  *     @arg RCC_AHBPeriph_ETH_MAC   
  *     @arg RCC_AHBPeriph_ETH_MAC_Tx
  *     @arg RCC_AHBPeriph_ETH_MAC_Rx
  * 
  *   For @b other_STM32_devices, this parameter can be any combination of the 
  *   following values:        
  *     @arg RCC_AHBPeriph_DMA1
  *     @arg RCC_AHBPeriph_DMA2
  *     @arg RCC_AHBPeriph_SRAM
  *     @arg RCC_AHBPeriph_FLITF
  *     @arg RCC_AHBPeriph_CRC
  *     @arg RCC_AHBPeriph_FSMC
  *     @arg RCC_AHBPeriph_SDIO
  *   
  * @note SRAM and FLITF clock can be disabled only during sleep mode.
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_AHBPeriphClockCmd(mut RCC_AHBPeriph: uint32_t,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh12 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).AHBENR;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_AHBPeriph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh13 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).AHBENR;
        ::core::ptr::write_volatile(fresh13,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_AHBPeriph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
  *          RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
  *          RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
  *          RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
  *          RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
  *          RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17,
  *          RCC_APB2Periph_TIM9, RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11     
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB2PeriphClockCmd(mut RCC_APB2Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh14 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh14,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh15 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh15,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to gates its clock.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
  *          RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
  *          RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
  *          RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4, 
  *          RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
  *          RCC_APB1Periph_USB, RCC_APB1Periph_CAN1, RCC_APB1Periph_BKP,
  *          RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_CEC,
  *          RCC_APB1Periph_TIM12, RCC_APB1Periph_TIM13, RCC_APB1Periph_TIM14
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB1PeriphClockCmd(mut RCC_APB1Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh16 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1ENR;
        ::core::ptr::write_volatile(fresh16,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh17 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1ENR;
        ::core::ptr::write_volatile(fresh17,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    };
}
/* STM32F10X_CL */
/* *
  * @brief  Forces or releases High Speed APB (APB2) peripheral reset.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to reset.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
  *          RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
  *          RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
  *          RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
  *          RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
  *          RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17,
  *          RCC_APB2Periph_TIM9, RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11  
  * @param  NewState: new state of the specified peripheral reset.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB2PeriphResetCmd(mut RCC_APB2Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh18 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2RSTR;
        ::core::ptr::write_volatile(fresh18,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh19 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2RSTR;
        ::core::ptr::write_volatile(fresh19,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Forces or releases Low Speed APB (APB1) peripheral reset.
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to reset.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
  *          RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
  *          RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
  *          RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4, 
  *          RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
  *          RCC_APB1Periph_USB, RCC_APB1Periph_CAN1, RCC_APB1Periph_BKP,
  *          RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_CEC,
  *          RCC_APB1Periph_TIM12, RCC_APB1Periph_TIM13, RCC_APB1Periph_TIM14  
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB1PeriphResetCmd(mut RCC_APB1Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh20 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1RSTR;
        ::core::ptr::write_volatile(fresh20,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh21 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1RSTR;
        ::core::ptr::write_volatile(fresh21,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Forces or releases the Backup domain reset.
  * @param  NewState: new state of the Backup domain reset.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_BackupResetCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
                                                                                                                           as
                                                                                                                           libc::c_int
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000
                                                                                                                                                          as
                                                                                                                                                          libc::c_int
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x20
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_int
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_int
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x10
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
  * @brief  Enables or disables the Clock Security System.
  * @param  NewState: new state of the Clock Security System..
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ClockSecuritySystemCmd(mut NewState:
                                                        FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x20000
                                                                                            as
                                                                                            libc::c_int
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000
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
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x13
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
  * @brief  Selects the clock source to output on MCO pin.
  * @param  RCC_MCO: specifies the clock source to output.
  *   
  *   For @b STM32_Connectivity_line_devices, this parameter can be one of the
  *   following values:       
  *     @arg RCC_MCO_NoClock: No clock selected
  *     @arg RCC_MCO_SYSCLK: System clock selected
  *     @arg RCC_MCO_HSI: HSI oscillator clock selected
  *     @arg RCC_MCO_HSE: HSE oscillator clock selected
  *     @arg RCC_MCO_PLLCLK_Div2: PLL clock divided by 2 selected
  *     @arg RCC_MCO_PLL2CLK: PLL2 clock selected                     
  *     @arg RCC_MCO_PLL3CLK_Div2: PLL3 clock divided by 2 selected   
  *     @arg RCC_MCO_XT1: External 3-25 MHz oscillator clock selected  
  *     @arg RCC_MCO_PLL3CLK: PLL3 clock selected 
  * 
  *   For  @b other_STM32_devices, this parameter can be one of the following values:        
  *     @arg RCC_MCO_NoClock: No clock selected
  *     @arg RCC_MCO_SYSCLK: System clock selected
  *     @arg RCC_MCO_HSI: HSI oscillator clock selected
  *     @arg RCC_MCO_HSE: HSE oscillator clock selected
  *     @arg RCC_MCO_PLLCLK_Div2: PLL clock divided by 2 selected
  *   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_MCOConfig(mut RCC_MCO: uint8_t) {
    /* Check the parameters */
    /* Perform Byte access to MCO bits to select the MCO source */
    ::core::ptr::write_volatile(0x40021007 as libc::c_int as uint32_t as
                                    *mut uint8_t, RCC_MCO);
}
/* *
  * @brief  Checks whether the specified RCC flag is set or not.
  * @param  RCC_FLAG: specifies the flag to check.
  *   
  *   For @b STM32_Connectivity_line_devices, this parameter can be one of the
  *   following values:
  *     @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready
  *     @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
  *     @arg RCC_FLAG_PLLRDY: PLL clock ready
  *     @arg RCC_FLAG_PLL2RDY: PLL2 clock ready      
  *     @arg RCC_FLAG_PLL3RDY: PLL3 clock ready                           
  *     @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
  *     @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
  *     @arg RCC_FLAG_PINRST: Pin reset
  *     @arg RCC_FLAG_PORRST: POR/PDR reset
  *     @arg RCC_FLAG_SFTRST: Software reset
  *     @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
  *     @arg RCC_FLAG_WWDGRST: Window Watchdog reset
  *     @arg RCC_FLAG_LPWRRST: Low Power reset
  * 
  *   For @b other_STM32_devices, this parameter can be one of the following values:        
  *     @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready
  *     @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
  *     @arg RCC_FLAG_PLLRDY: PLL clock ready
  *     @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
  *     @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
  *     @arg RCC_FLAG_PINRST: Pin reset
  *     @arg RCC_FLAG_PORRST: POR/PDR reset
  *     @arg RCC_FLAG_SFTRST: Software reset
  *     @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
  *     @arg RCC_FLAG_WWDGRST: Window Watchdog reset
  *     @arg RCC_FLAG_LPWRRST: Low Power reset
  *   
  * @retval The new state of RCC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetFlagStatus(mut RCC_FLAG: uint8_t)
 -> FlagStatus {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    let mut statusreg: uint32_t = 0 as libc::c_int as uint32_t;
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Get the RCC register index */
    tmp = (RCC_FLAG as libc::c_int >> 5 as libc::c_int) as uint32_t;
    if tmp == 1 as libc::c_int as libc::c_uint {
        /* The flag to check is in CR register */
        statusreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CR
    } else if tmp == 2 as libc::c_int as libc::c_uint {
        /* The flag to check is in BDCR register */
        statusreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).BDCR
    } else {
        /* The flag to check is in CSR register */
        statusreg =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x1000
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CSR
    }
    /* Get the flag position */
    tmp =
        (RCC_FLAG as libc::c_int &
             0x1f as libc::c_int as uint8_t as libc::c_int) as uint32_t;
    if statusreg & (1 as libc::c_int as uint32_t) << tmp !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the flag status */
    return bitstatus;
}
/* *
  * @brief  Clears the RCC reset flags.
  * @note   The reset flags are: RCC_FLAG_PINRST, RCC_FLAG_PORRST, RCC_FLAG_SFTRST,
  *   RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LPWRRST
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ClearFlag() {
    /* Set RMVF bit to clear the reset flags */
    let ref mut fresh22 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CSR;
    ::core::ptr::write_volatile(fresh22,
                                (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1000000 as libc::c_int as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Checks whether the specified RCC interrupt has occurred or not.
  * @param  RCC_IT: specifies the RCC interrupt source to check.
  *   
  *   For @b STM32_Connectivity_line_devices, this parameter can be one of the
  *   following values:
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *     @arg RCC_IT_PLL2RDY: PLL2 ready interrupt 
  *     @arg RCC_IT_PLL3RDY: PLL3 ready interrupt                      
  *     @arg RCC_IT_CSS: Clock Security System interrupt
  * 
  *   For @b other_STM32_devices, this parameter can be one of the following values:        
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *     @arg RCC_IT_CSS: Clock Security System interrupt
  *   
  * @retval The new state of RCC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetITStatus(mut RCC_IT: uint8_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified RCC interrupt */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x20000 as libc::c_int as
                                          libc::c_uint).wrapping_add(0x1000 as
                                                                         libc::c_int
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CIR & RCC_IT as libc::c_uint !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the RCC_IT status */
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_rcc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the RCC firmware 
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
/* * @addtogroup RCC
  * @{
  */
/* * @defgroup RCC_Exported_Types
  * @{
  */
/* !< returns SYSCLK clock frequency expressed in Hz */
/* !< returns HCLK clock frequency expressed in Hz */
/* !< returns PCLK1 clock frequency expressed in Hz */
/* !< returns PCLK2 clock frequency expressed in Hz */
/* !< returns ADCCLK clock frequency expressed in Hz */
/* *
  * @}
  */
/* * @defgroup RCC_Exported_Constants
  * @{
  */
/* * @defgroup HSE_configuration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PLL_entry_clock_source 
  * @{
  */
/* STM32F10X_CL */
/* *
  * @}
  */
/* * @defgroup PLL_multiplication_factor 
  * @{
  */
/* STM32F10X_CL */
/* *
  * @}
  */
/* * @defgroup PREDIV1_division_factor
  * @{
  */
/* *
  * @}
  */
/* * @defgroup PREDIV1_clock_source
  * @{
  */
/* *
  * @}
  */
/* STM32F10X_CL */
/* * @defgroup System_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup AHB_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup APB1_APB2_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Interrupt_source 
  * @{
  */
/* STM32F10X_CL */
/* *
  * @}
  */
/* * @defgroup USB_Device_clock_source 
  * @{
  */
/* *
  * @}
  */
/* STM32F10X_CL */
/* STM32F10X_CL */
/* * @defgroup ADC_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup LSE_configuration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup AHB_peripheral 
  * @{
  */
/* STM32F10X_CL */
/* *
  * @}
  */
/* * @defgroup APB2_peripheral 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup APB1_peripheral 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup Clock_source_to_output_on_MCO_pin 
  * @{
  */
/* STM32F10X_CL */
/* *
  * @}
  */
/* * @defgroup RCC_Flag 
  * @{
  */
/* STM32F10X_CL */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup RCC_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Exported_Functions
  * @{
  */
/* STM32F10X_CL */
/* STM32F10X_CL */
/* STM32F10X_CL */
/* STM32F10X_CL */
/* *
  * @brief  Clears the RCC's interrupt pending bits.
  * @param  RCC_IT: specifies the interrupt pending bit to clear.
  *   
  *   For @b STM32_Connectivity_line_devices, this parameter can be any combination
  *   of the following values:
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *     @arg RCC_IT_PLL2RDY: PLL2 ready interrupt 
  *     @arg RCC_IT_PLL3RDY: PLL3 ready interrupt                      
  *     @arg RCC_IT_CSS: Clock Security System interrupt
  * 
  *   For @b other_STM32_devices, this parameter can be any combination of the
  *   following values:        
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *   
  *     @arg RCC_IT_CSS: Clock Security System interrupt
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ClearITPendingBit(mut RCC_IT: uint8_t) {
    /* Check the parameters */
    /* Perform Byte access to RCC_CIR[23:16] bits to clear the selected interrupt
     pending bits */
    ::core::ptr::write_volatile(0x4002100a as libc::c_int as uint32_t as
                                    *mut uint8_t, RCC_IT);
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
