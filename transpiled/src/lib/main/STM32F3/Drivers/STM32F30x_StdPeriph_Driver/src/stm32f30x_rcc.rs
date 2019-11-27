use core;
use libc;
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
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
#[derive ( Copy, Clone )]
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
    pub AHBRSTR: uint32_t,
    pub CFGR2: uint32_t,
    pub CFGR3: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RCC_ClocksTypeDef {
    pub SYSCLK_Frequency: uint32_t,
    pub HCLK_Frequency: uint32_t,
    pub PCLK1_Frequency: uint32_t,
    pub PCLK2_Frequency: uint32_t,
    pub ADC12CLK_Frequency: uint32_t,
    pub ADC34CLK_Frequency: uint32_t,
    pub I2C1CLK_Frequency: uint32_t,
    pub I2C2CLK_Frequency: uint32_t,
    pub I2C3CLK_Frequency: uint32_t,
    pub TIM1CLK_Frequency: uint32_t,
    pub HRTIM1CLK_Frequency: uint32_t,
    pub TIM8CLK_Frequency: uint32_t,
    pub USART1CLK_Frequency: uint32_t,
    pub USART2CLK_Frequency: uint32_t,
    pub USART3CLK_Frequency: uint32_t,
    pub UART4CLK_Frequency: uint32_t,
    pub UART5CLK_Frequency: uint32_t,
    pub TIM15CLK_Frequency: uint32_t,
    pub TIM16CLK_Frequency: uint32_t,
    pub TIM17CLK_Frequency: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static mut APBAHBPrescTable: [uint8_t; 16] =
    [0i32 as uint8_t, 0i32 as uint8_t, 0i32 as uint8_t, 0i32 as uint8_t,
     1i32 as uint8_t, 2i32 as uint8_t, 3i32 as uint8_t, 4i32 as uint8_t,
     1i32 as uint8_t, 2i32 as uint8_t, 3i32 as uint8_t, 4i32 as uint8_t,
     6i32 as uint8_t, 7i32 as uint8_t, 8i32 as uint8_t, 9i32 as uint8_t];
static mut ADCPrescTable: [uint16_t; 16] =
    [1i32 as uint16_t, 2i32 as uint16_t, 4i32 as uint16_t, 6i32 as uint16_t,
     8i32 as uint16_t, 10i32 as uint16_t, 12i32 as uint16_t,
     16i32 as uint16_t, 32i32 as uint16_t, 64i32 as uint16_t,
     128i32 as uint16_t, 256i32 as uint16_t, 0i32 as uint16_t,
     0i32 as uint16_t, 0i32 as uint16_t, 0i32 as uint16_t];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup RCC_Private_Functions
  * @{
  */
/* * @defgroup RCC_Group1 Internal and external clocks, PLL, CSS and MCO configuration functions
 *  @brief   Internal and external clocks, PLL, CSS and MCO configuration functions 
 *
@verbatim   
 ===============================================================================
 ##### Internal-external clocks, PLL, CSS and MCO configuration functions #####
 ===============================================================================  
    [..] This section provides functions allowing to configure the internal/external 
         clocks, PLL, CSS and MCO.
         (#) HSI (high-speed internal), 8 MHz factory-trimmed RC used directly 
             or through the PLL as System clock source.
             The HSI clock can be used also to clock the USART and I2C peripherals.
         (#) LSI (low-speed internal), 40 KHz low consumption RC used as IWDG and/or RTC
             clock source.
         (#) HSE (high-speed external), 4 to 32 MHz crystal oscillator used directly or
             through the PLL as System clock source. Can be used also as RTC clock source.
         (#) LSE (low-speed external), 32 KHz oscillator used as RTC clock source.
             LSE can be used also to clock the USART peripherals.
         (#) PLL (clocked by HSI or HSE), for System clock.
         (#) CSS (Clock security system), once enabled and if a HSE clock failure occurs 
             (HSE used directly or through PLL as System clock source), the System clock
             is automatically switched to HSI and an interrupt is generated if enabled. 
             The interrupt is linked to the Cortex-M4 NMI (Non-Maskable Interrupt) 
             exception vector.   
         (#) MCO (microcontroller clock output), used to output SYSCLK, HSI, HSE, LSI, LSE,
             PLL clock on PA8 pin.

@endverbatim
  * @{
  */
/* *
  * @brief  Resets the RCC clock configuration to the default reset state.
  * @note   The default reset state of the clock configuration is given below:
  * @note     HSI ON and used as system clock source 
  * @note     HSE and PLL OFF
  * @note     AHB, APB1 and APB2 prescalers set to 1.
  * @note     CSS and MCO OFF
  * @note     All interrupts disabled
  * @note   However, this function doesn't modify the configuration of the
  * @note     Peripheral clocks
  * @note     LSI, LSE and RTC clocks                  
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_DeInit() {
    /* Set HSION bit */
    let ref mut fresh0 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x1i32 as uint32_t) as
                                    uint32_t as uint32_t);
    /* Reset SW[1:0], HPRE[3:0], PPRE[2:0] and MCOSEL[2:0] bits */
    let ref mut fresh1 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh1,
                                (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & 0xf8ffc000u32) as
                                    uint32_t as uint32_t);
    /* Reset HSEON, CSSON and PLLON bits */
    let ref mut fresh2 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & 0xfef6ffffu32) as
                                    uint32_t as uint32_t);
    /* Reset HSEBYP bit */
    let ref mut fresh3 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & 0xfffbffffu32) as
                                    uint32_t as uint32_t);
    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
    let ref mut fresh4 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & 0xff80ffffu32) as
                                    uint32_t as uint32_t);
    /* Reset PREDIV1[3:0] and ADCPRE[13:4] bits */
    let ref mut fresh5 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR2;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & 0xffffc000u32) as
                                    uint32_t as uint32_t);
    /* Reset USARTSW[1:0], I2CSW and TIMSW bits */
    let ref mut fresh6 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR3;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     0xf00eccci32 as uint32_t) as uint32_t as
                                    uint32_t);
    /* Disable all interrupts */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CIR as
                                    *mut uint32_t, 0i32 as uint32_t);
}
/* *
  * @brief  Configures the External High Speed oscillator (HSE).
  * @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
  *         software should wait on HSERDY flag to be set indicating that HSE clock
  *         is stable and can be used to clock the PLL and/or system clock.
  * @note   HSE state can not be changed if it is used directly or through the
  *         PLL as system clock. In this case, you have to select another source
  *         of the system clock then change the HSE state (ex. disable it).
  * @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.         
  * @note   This function resets the CSSON bit, so if the Clock security system(CSS)
  *         was previously enabled you have to enable it again after calling this
  *         function.
  * @param  RCC_HSE: specifies the new state of the HSE.
  *   This parameter can be one of the following values:
  *     @arg RCC_HSE_OFF: turn OFF the HSE oscillator, HSERDY flag goes low after
  *                       6 HSE oscillator clock cycles.
  *     @arg RCC_HSE_ON: turn ON the HSE oscillator
  *     @arg RCC_HSE_Bypass: HSE oscillator bypassed with external clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HSEConfig(mut RCC_HSE: uint8_t) {
    /* Check the parameters */
    /* Reset HSEON and HSEBYP bits before configuring the HSE ------------------*/
    ::core::ptr::write_volatile(0x40021002i32 as uint32_t as *mut uint8_t,
                                0i32 as uint8_t);
    /* Set the new HSE configuration -------------------------------------------*/
    ::core::ptr::write_volatile(0x40021002i32 as uint32_t as *mut uint8_t,
                                RCC_HSE);
}
/* *
  * @brief  Waits for HSE start-up.
  * @note   This function waits on HSERDY flag to be set and return SUCCESS if 
  *         this flag is set, otherwise returns ERROR if the timeout is reached 
  *         and this flag is not set. The timeout value is defined by the constant
  *         HSE_STARTUP_TIMEOUT in stm32f30x.h file. You can tailor it depending
  *         on the HSE crystal used in your application. 
  * @param  None
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: HSE oscillator is stable and ready to use
  *          - ERROR: HSE oscillator not yet ready
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_WaitForHSEStartUp() -> ErrorStatus {
    let mut StartUpCounter: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    let mut HSEStatus: FlagStatus = RESET;
    loop 
         /* Wait till HSE is ready and if timeout is reached exit */
         {
        HSEStatus = RCC_GetFlagStatus(0x11i32 as uint8_t);
        ::core::ptr::write_volatile(&mut StartUpCounter as *mut uint32_t,
                                    ::core::ptr::read_volatile::<uint32_t>(&StartUpCounter
                                                                               as
                                                                               *const uint32_t).wrapping_add(1));
        if !(StartUpCounter != 0x5000i32 as uint16_t as libc::c_uint &&
                 HSEStatus as libc::c_uint ==
                     RESET as libc::c_int as libc::c_uint) {
            break ;
        }
    }
    if RCC_GetFlagStatus(0x11i32 as uint8_t) as libc::c_uint !=
           RESET as libc::c_int as libc::c_uint {
        status = SUCCESS
    } else { status = ERROR }
    return status;
}
/* *
  * @brief  Adjusts the Internal High Speed oscillator (HSI) calibration value.
  * @note   The calibration is used to compensate for the variations in voltage
  *         and temperature that influence the frequency of the internal HSI RC.
  *         Refer to the Application Note AN3300 for more details on how to  
  *         calibrate the HSI.
  * @param  HSICalibrationValue: specifies the HSI calibration trimming value.
  *         This parameter must be a number between 0 and 0x1F.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_AdjustHSICalibrationValue(mut HSICalibrationValue:
                                                           uint8_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CR;
    /* Clear HSITRIM[4:0] bits */
    tmpreg &= !(0xf8i32 as uint32_t);
    /* Set the HSITRIM[4:0] bits according to HSICalibrationValue value */
    tmpreg |= (HSICalibrationValue as uint32_t) << 3i32;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the Internal High Speed oscillator (HSI).
  * @note   After enabling the HSI, the application software should wait on 
  *         HSIRDY flag to be set indicating that HSI clock is stable and can
  *         be used to clock the PLL and/or system clock.
  * @note   HSI can not be stopped if it is used directly or through the PLL
  *         as system clock. In this case, you have to select another source 
  *         of the system clock then stop the HSI.
  * @note   The HSI is stopped by hardware when entering STOP and STANDBY modes. 
  * @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
  *         clock cycles.    
  * @param  NewState: new state of the HSI.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HSICmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Configures the External Low Speed oscillator (LSE).
  * @note     As the LSE is in the Backup domain and write access is denied to this
  *           domain after reset, you have to enable write access using 
  *           PWR_BackupAccessCmd(ENABLE) function before to configure the LSE
  *           (to be done once after reset).
  * @note     After enabling the LSE (RCC_LSE_ON or RCC_LSE_Bypass), the application
  *           software should wait on LSERDY flag to be set indicating that LSE clock
  *           is stable and can be used to clock the RTC.
  * @param  RCC_LSE: specifies the new state of the LSE.
  *   This parameter can be one of the following values:
  *     @arg RCC_LSE_OFF: turn OFF the LSE oscillator, LSERDY flag goes low after
  *                       6 LSE oscillator clock cycles.
  *     @arg RCC_LSE_ON: turn ON the LSE oscillator
  *     @arg RCC_LSE_Bypass: LSE oscillator bypassed with external clock
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_LSEConfig(mut RCC_LSE: uint32_t) {
    /* Check the parameters */
    /* Reset LSEON and LSEBYP bits before configuring the LSE ------------------*/
  /* Reset LSEON bit */
    let ref mut fresh7 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh7,
                                (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x1i32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Reset LSEBYP bit */
    let ref mut fresh8 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh8,
                                (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x4i32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Configure LSE */
    let ref mut fresh9 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh9,
                                (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_LSE) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Configures the External Low Speed oscillator (LSE) drive capability.
  * @param  RCC_LSEDrive: specifies the new state of the LSE drive capability.
  *   This parameter can be one of the following values:
  *     @arg RCC_LSEDrive_Low: LSE oscillator low drive capability.
  *     @arg RCC_LSEDrive_MediumLow: LSE oscillator medium low drive capability.
  *     @arg RCC_LSEDrive_MediumHigh: LSE oscillator medium high drive capability.
  *     @arg RCC_LSEDrive_High: LSE oscillator high drive capability.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_LSEDriveConfig(mut RCC_LSEDrive: uint32_t) {
    /* Check the parameters */
    /* Clear LSEDRV[1:0] bits */
    let ref mut fresh10 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh10,
                                (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x18i32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Set the LSE Drive */
    let ref mut fresh11 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh11,
                                (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_LSEDrive) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables or disables the Internal Low Speed oscillator (LSI).  
  * @note   After enabling the LSI, the application software should wait on 
  *         LSIRDY flag to be set indicating that LSI clock is stable and can
  *         be used to clock the IWDG and/or the RTC.
  * @note   LSI can not be disabled if the IWDG is running.  
  * @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
  *         clock cycles.
  * @param  NewState: new state of the LSI.
  *         This parameter can be: ENABLE or DISABLE. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_LSICmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x24i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Configures the PLL clock source and multiplication factor.
  * @note   This function must be used only when the PLL is disabled.
  * @note   The minimum input clock frequency for PLL is 2 MHz (when using HSE as
  *         PLL source).   
  * @param  RCC_PLLSource: specifies the PLL entry clock source.
  *   This parameter can be one of the following values:
  *     @arg RCC_PLLSource_HSI_Div2: HSI oscillator clock divided by 2 selected as
  *         PLL clock entry
  *     @arg RCC_PLLSource_PREDIV1: PREDIV1 clock selected as PLL clock source              
  * @param  RCC_PLLMul: specifies the PLL multiplication factor, which drive the PLLVCO clock
  *   This parameter can be RCC_PLLMul_x where x:[2,16] 
  *                                               
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLConfig(mut RCC_PLLSource: uint32_t,
                                       mut RCC_PLLMul: uint32_t) {
    /* Check the parameters */
    /* Clear PLL Source [16] and Multiplier [21:18] bits */
    let ref mut fresh12 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh12,
                                (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x3c0000i32 as uint32_t |
                                           0x10000i32 as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Set the PLL Source and Multiplier */
    let ref mut fresh13 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (RCC_PLLSource | RCC_PLLMul)) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Enables or disables the PLL.
  * @note   After enabling the PLL, the application software should wait on 
  *         PLLRDY flag to be set indicating that PLL clock is stable and can
  *         be used as system clock source.
  * @note   The PLL can not be disabled if it is used as system clock source
  * @note   The PLL is disabled by hardware when entering STOP and STANDBY modes.    
  * @param  NewState: new state of the PLL.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PLLCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x18i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Configures the PREDIV1 division factor.
  * @note   This function must be used only when the PLL is disabled.
  * @param  RCC_PREDIV1_Div: specifies the PREDIV1 clock division factor.
  *         This parameter can be RCC_PREDIV1_Divx where x:[1,16]
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PREDIV1Config(mut RCC_PREDIV1_Div: uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR2;
    /* Clear PREDIV1[3:0] bits */
    tmpreg &= !(0xfi32 as uint32_t);
    /* Set the PREDIV1 division factor */
    tmpreg |= RCC_PREDIV1_Div;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR2 as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables or disables the Clock Security System.
  * @note   If a failure is detected on the HSE oscillator clock, this oscillator
  *         is automatically disabled and an interrupt is generated to inform the
  *         software about the failure (Clock Security System Interrupt, CSSI),
  *         allowing the MCU to perform rescue operations. The CSSI is linked to 
  *         the Cortex-M4 NMI (Non-Maskable Interrupt) exception vector.  
  * @param  NewState: new state of the Clock Security System.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ClockSecuritySystemCmd(mut NewState:
                                                        FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x13i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Selects the clock source to output on MCO pin (PA8).
  * @note   PA8 should be configured in alternate function mode.
  * @param  RCC_MCOSource: specifies the clock source to output.
  *          This parameter can be one of the following values:
  *            @arg RCC_MCOSource_NoClock: No clock selected.
  *            @arg RCC_MCOSource_HSI14: HSI14 oscillator clock selected.
  *            @arg RCC_MCOSource_LSI: LSI oscillator clock selected.
  *            @arg RCC_MCOSource_LSE: LSE oscillator clock selected.
  *            @arg RCC_MCOSource_SYSCLK: System clock selected.
  *            @arg RCC_MCOSource_HSI: HSI oscillator clock selected.
  *            @arg RCC_MCOSource_HSE: HSE oscillator clock selected.
  *            @arg RCC_MCOSource_PLLCLK_Div2: PLL clock divided by 2 selected.
  *            @arg RCC_MCOSource_PLLCLK: PLL clock selected.
  *            @arg RCC_MCOSource_HSI48: HSI48 clock selected.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_MCOConfig(mut RCC_MCOSource: uint8_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get CFGR value */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear MCO[3:0] bits */
    tmpreg &= !(0x7000000i32 as uint32_t | 0x80000000u32);
    /* Set the RCC_MCOSource */
    tmpreg |= ((RCC_MCOSource as libc::c_int) << 24i32) as libc::c_uint;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* STM32F303xC */
/* *
  * @}
  */
/* * @defgroup RCC_Group2 System AHB, APB1 and APB2 busses clocks configuration functions
 *  @brief   System, AHB and APB busses clocks configuration functions
 *
@verbatim   
 ===============================================================================
  ##### System, AHB, APB1 and APB2 busses clocks configuration functions #####
 ===============================================================================  
    [..] This section provide functions allowing to configure the System, AHB, APB1 and 
         APB2 busses clocks.
         (#) Several clock sources can be used to drive the System clock (SYSCLK): HSI,
             HSE and PLL.
             The AHB clock (HCLK) is derived from System clock through configurable prescaler
             and used to clock the CPU, memory and peripherals mapped on AHB bus (DMA and GPIO).
             APB1 (PCLK1) and APB2 (PCLK2) clocks are derived from AHB clock through 
             configurable prescalers and used to clock the peripherals mapped on these busses.
             You can use "RCC_GetClocksFreq()" function to retrieve the frequencies of these clocks.

         (#) The maximum frequency of the SYSCLK, HCLK, PCLK1 and PCLK2 is 72 MHz.
             Depending on the maximum frequency, the FLASH wait states (WS) should be 
             adapted accordingly:
        +---------------------------------+
        |  Wait states  |   HCLK clock    |
        |   (Latency)   | frequency (MHz) |
        |-------------- |-----------------|             
        |0WS(1CPU cycle)| 0 < HCLK <= 24  |
        |---------------|-----------------| 
        |1WS(2CPU cycle)|24 < HCLK <=48   |
        |---------------|-----------------| 
        |2WS(3CPU cycle)|48 < HCLK <= 72  |
        +---------------------------------+

         (#) After reset, the System clock source is the HSI (8 MHz) with 0 WS and 
             prefetch is disabled.
        [..]
         (@) All the peripheral clocks are derived from the System clock (SYSCLK) 
             except:
             (+@) The FLASH program/erase clock  which is always HSI 8MHz clock.
             (+@) The USB 48 MHz clock which is derived from the PLL VCO clock.
             (+@) The USART clock which can be derived as well from HSI 8MHz, LSI or LSE.
             (+@) The I2C clock which can be derived as well from HSI 8MHz clock.
             (+@) The ADC clock which is derived from PLL output.
             (+@) The RTC clock which is derived from the LSE, LSI or 1 MHz HSE_RTC 
                  (HSE divided by a programmable prescaler). The System clock (SYSCLK) 
                  frequency must be higher or equal to the RTC clock frequency.
             (+@) IWDG clock which is always the LSI clock.
    [..] It is recommended to use the following software sequences to tune the number
         of wait states needed to access the Flash memory with the CPU frequency (HCLK).
         (+) Increasing the CPU frequency
            (++) Program the Flash Prefetch buffer, using "FLASH_PrefetchBufferCmd(ENABLE)" 
                 function
            (++) Check that Flash Prefetch buffer activation is taken into account by 
                 reading FLASH_ACR using the FLASH_GetPrefetchBufferStatus() function
            (++) Program Flash WS to 1 or 2, using "FLASH_SetLatency()" function
            (++) Check that the new number of WS is taken into account by reading FLASH_ACR
            (++) Modify the CPU clock source, using "RCC_SYSCLKConfig()" function
            (++) If needed, modify the CPU clock prescaler by using "RCC_HCLKConfig()" function
            (++) Check that the new CPU clock source is taken into account by reading 
                 the clock source status, using "RCC_GetSYSCLKSource()" function 
         (+) Decreasing the CPU frequency
            (++) Modify the CPU clock source, using "RCC_SYSCLKConfig()" function
            (++) If needed, modify the CPU clock prescaler by using "RCC_HCLKConfig()" function
            (++) Check that the new CPU clock source is taken into account by reading 
                 the clock source status, using "RCC_GetSYSCLKSource()" function
            (++) Program the new number of WS, using "FLASH_SetLatency()" function
            (++) Check that the new number of WS is taken into account by reading FLASH_ACR
            (++) Disable the Flash Prefetch buffer using "FLASH_PrefetchBufferCmd(DISABLE)" 
                 function
            (++) Check that Flash Prefetch buffer deactivation is taken into account by reading FLASH_ACR
                 using the FLASH_GetPrefetchBufferStatus() function.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the system clock (SYSCLK).
  * @note     The HSI is used (enabled by hardware) as system clock source after
  *           startup from Reset, wake-up from STOP and STANDBY mode, or in case
  *           of failure of the HSE used directly or indirectly as system clock
  *           (if the Clock Security System CSS is enabled).
  * @note     A switch from one clock source to another occurs only if the target
  *           clock source is ready (clock stable after startup delay or PLL locked). 
  *           If a clock source which is not yet ready is selected, the switch will
  *           occur when the clock source will be ready. 
  *           You can use RCC_GetSYSCLKSource() function to know which clock is
  *           currently used as system clock source.  
  * @param  RCC_SYSCLKSource: specifies the clock source used as system clock source 
  *   This parameter can be one of the following values:
  *     @arg RCC_SYSCLKSource_HSI:    HSI selected as system clock source
  *     @arg RCC_SYSCLKSource_HSE:    HSE selected as system clock source
  *     @arg RCC_SYSCLKSource_PLLCLK: PLL selected as system clock source
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_SYSCLKConfig(mut RCC_SYSCLKSource: uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear SW[1:0] bits */
    tmpreg &= !(0x3i32 as uint32_t);
    /* Set SW[1:0] bits according to RCC_SYSCLKSource value */
    tmpreg |= RCC_SYSCLKSource;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Returns the clock source used as system clock.
  * @param  None
  * @retval The clock source used as system clock. The returned value can be one 
  *         of the following values:
  *              - 0x00: HSI used as system clock
  *              - 0x04: HSE used as system clock  
  *              - 0x08: PLL used as system clock
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetSYSCLKSource() -> uint8_t {
    return ((*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR & 0xci32 as uint32_t) as
               uint8_t;
}
/* *
  * @brief  Configures the AHB clock (HCLK).
  * @note   Depending on the device voltage range, the software has to set correctly
  *         these bits to ensure that the system frequency does not exceed the
  *         maximum allowed frequency (for more details refer to section above
  *         "CPU, AHB and APB busses clocks configuration functions").
  * @param  RCC_SYSCLK: defines the AHB clock divider. This clock is derived from 
  *                     the system clock (SYSCLK).
  *   This parameter can be one of the following values:
  *     @arg RCC_SYSCLK_Div1:   AHB clock = SYSCLK
  *     @arg RCC_SYSCLK_Div2:   AHB clock = SYSCLK/2
  *     @arg RCC_SYSCLK_Div4:   AHB clock = SYSCLK/4
  *     @arg RCC_SYSCLK_Div8:   AHB clock = SYSCLK/8
  *     @arg RCC_SYSCLK_Div16:  AHB clock = SYSCLK/16
  *     @arg RCC_SYSCLK_Div64:  AHB clock = SYSCLK/64
  *     @arg RCC_SYSCLK_Div128: AHB clock = SYSCLK/128
  *     @arg RCC_SYSCLK_Div256: AHB clock = SYSCLK/256
  *     @arg RCC_SYSCLK_Div512: AHB clock = SYSCLK/512
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HCLKConfig(mut RCC_SYSCLK: uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear HPRE[3:0] bits */
    tmpreg &= !(0xf0i32 as uint32_t);
    /* Set HPRE[3:0] bits according to RCC_SYSCLK value */
    tmpreg |= RCC_SYSCLK;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the Low Speed APB clock (PCLK1).
  * @param  RCC_HCLK: defines the APB1 clock divider. This clock is derived from 
  *         the AHB clock (HCLK).
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
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear PPRE1[2:0] bits */
    tmpreg &= !(0x700i32 as uint32_t);
    /* Set PPRE1[2:0] bits according to RCC_HCLK value */
    tmpreg |= RCC_HCLK;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Configures the High Speed APB clock (PCLK2).
  * @param  RCC_HCLK: defines the APB2 clock divider. This clock is derived from 
  *         the AHB clock (HCLK).
  *         This parameter can be one of the following values:
  *             @arg RCC_HCLK_Div1: APB2 clock = HCLK
  *             @arg RCC_HCLK_Div2: APB2 clock = HCLK/2
  *             @arg RCC_HCLK_Div4: APB2 clock = HCLK/4
  *             @arg RCC_HCLK_Div8: APB2 clock = HCLK/8
  *             @arg RCC_HCLK_Div16: APB2 clock = HCLK/16
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_PCLK2Config(mut RCC_HCLK: uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpreg =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR;
    /* Clear PPRE2[2:0] bits */
    tmpreg &= !(0x3800i32 as uint32_t);
    /* Set PPRE2[2:0] bits according to RCC_HCLK value */
    tmpreg |= RCC_HCLK << 3i32;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x20000i32
                                                                        as
                                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                                       as
                                                                                                       libc::c_uint)
                                            as *mut RCC_TypeDef)).CFGR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Returns the frequencies of the System, AHB, APB2 and APB1 busses clocks.
  * 
  *  @note    This function returns the frequencies of :
  *           System, AHB, APB2 and APB1 busses clocks, ADC1/2/3/4 clocks, 
  *           USART1/2/3/4/5 clocks, I2C1/2 clocks and TIM1/8 Clocks.
  *                         
  * @note     The frequency returned by this function is not the real frequency
  *           in the chip. It is calculated based on the predefined constant and
  *           the source selected by RCC_SYSCLKConfig().
  *                                              
  * @note      If SYSCLK source is HSI, function returns constant HSI_VALUE(*)
  *                                              
  * @note      If SYSCLK source is HSE, function returns constant HSE_VALUE(**)
  *                          
  * @note      If SYSCLK source is PLL, function returns constant HSE_VALUE(**) 
  *             or HSI_VALUE(*) multiplied by the PLL factors.
  *         
  * @note     (*) HSI_VALUE is a constant defined in stm32f30x.h file (default value
  *               8 MHz) but the real value may vary depending on the variations
  *               in voltage and temperature, refer to RCC_AdjustHSICalibrationValue().   
  *    
  * @note     (**) HSE_VALUE is a constant defined in stm32f30x.h file (default value
  *                8 MHz), user has to ensure that HSE_VALUE is same as the real
  *                frequency of the crystal used. Otherwise, this function may
  *                return wrong result.
  *                
  * @note     The result of this function could be not correct when using fractional
  *           value for HSE crystal.   
  *             
  * @param  RCC_Clocks: pointer to a RCC_ClocksTypeDef structure which will hold 
  *         the clocks frequencies. 
  *     
  * @note     This function can be used by the user application to compute the 
  *           baudrate for the communication peripherals or configure other parameters.
  * @note     Each time SYSCLK, HCLK, PCLK1 and/or PCLK2 clock changes, this function
  *           must be called to update the structure's field. Otherwise, any
  *           configuration based on this function will be incorrect.
  *    
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetClocksFreq(mut RCC_Clocks:
                                               *mut RCC_ClocksTypeDef) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    let mut pllmull: uint32_t = 0i32 as uint32_t;
    let mut pllsource: uint32_t = 0i32 as uint32_t;
    let mut prediv1factor: uint32_t = 0i32 as uint32_t;
    let mut presc: uint32_t = 0i32 as uint32_t;
    let mut pllclk: uint32_t = 0i32 as uint32_t;
    let mut apb2presc: uint32_t = 0i32 as uint32_t;
    let mut ahbpresc: uint32_t = 0i32 as uint32_t;
    /* Get SYSCLK source -------------------------------------------------------*/
    tmp =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0xci32 as uint32_t;
    match tmp {
        0 => {
            /* HSI used as system clock */
            (*RCC_Clocks).SYSCLK_Frequency = 8000000i32 as uint32_t
        }
        4 => {
            /* HSE used as system clock */
            (*RCC_Clocks).SYSCLK_Frequency = 8000000i32 as uint32_t
        }
        8 => {
            /* PLL used as system clock */
            /* Get PLL clock source and multiplication factor ----------------------*/
            pllmull =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR & 0x3c0000i32 as uint32_t;
            pllsource =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR & 0x10000i32 as uint32_t;
            pllmull = (pllmull >> 18i32).wrapping_add(2i32 as libc::c_uint);
            if pllsource == 0i32 as libc::c_uint {
                /* HSI oscillator clock divided by 2 selected as PLL clock entry */
                pllclk =
                    (8000000i32 as uint32_t >> 1i32).wrapping_mul(pllmull)
            } else {
                prediv1factor =
                    ((*((0x40000000i32 as
                             uint32_t).wrapping_add(0x20000i32 as
                                                        libc::c_uint).wrapping_add(0x1000i32
                                                                                       as
                                                                                       libc::c_uint)
                            as *mut RCC_TypeDef)).CFGR2 &
                         0xfi32 as
                             uint32_t).wrapping_add(1i32 as libc::c_uint);
                /* HSE oscillator clock selected as PREDIV1 clock entry */
                pllclk =
                    (8000000i32 as
                         libc::c_uint).wrapping_div(prediv1factor).wrapping_mul(pllmull)
            }
            (*RCC_Clocks).SYSCLK_Frequency = pllclk
        }
        _ => {
            /* HSI used as system clock */
            (*RCC_Clocks).SYSCLK_Frequency = 8000000i32 as uint32_t
        }
    }
    /* Compute HCLK, PCLK clocks frequencies -----------------------------------*/
  /* Get HCLK prescaler */
    tmp =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0xf0i32 as uint32_t;
    tmp = tmp >> 4i32;
    ahbpresc = APBAHBPrescTable[tmp as usize] as uint32_t;
    /* HCLK clock frequency */
    (*RCC_Clocks).HCLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency >> ahbpresc;
    /* Get PCLK1 prescaler */
    tmp =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0x700i32 as uint32_t;
    tmp = tmp >> 8i32;
    presc = APBAHBPrescTable[tmp as usize] as uint32_t;
    /* PCLK1 clock frequency */
    (*RCC_Clocks).PCLK1_Frequency = (*RCC_Clocks).HCLK_Frequency >> presc;
    /* Get PCLK2 prescaler */
    tmp =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR & 0x3800i32 as uint32_t;
    tmp = tmp >> 11i32;
    apb2presc = APBAHBPrescTable[tmp as usize] as uint32_t;
    /* PCLK2 clock frequency */
    (*RCC_Clocks).PCLK2_Frequency = (*RCC_Clocks).HCLK_Frequency >> apb2presc;
    /* Get ADC12CLK prescaler */
    tmp =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR2 & 0x1f0i32 as uint32_t;
    tmp = tmp >> 4i32;
    presc =
        ADCPrescTable[(tmp & 0xfi32 as libc::c_uint) as usize] as uint32_t;
    if tmp & 0x10i32 as libc::c_uint != 0i32 as libc::c_uint &&
           presc != 0i32 as libc::c_uint {
        /* ADC12CLK clock frequency is derived from PLL clock */
        (*RCC_Clocks).ADC12CLK_Frequency = pllclk.wrapping_div(presc)
    } else {
        /* ADC12CLK clock frequency is AHB clock */
        (*RCC_Clocks).ADC12CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    }
    /* Get ADC34CLK prescaler */
    tmp =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR2 & 0x3e00i32 as uint32_t;
    tmp = tmp >> 9i32;
    presc =
        ADCPrescTable[(tmp & 0xfi32 as libc::c_uint) as usize] as uint32_t;
    if tmp & 0x10i32 as libc::c_uint != 0i32 as libc::c_uint &&
           presc != 0i32 as libc::c_uint {
        /* ADC34CLK clock frequency is derived from PLL clock */
        (*RCC_Clocks).ADC34CLK_Frequency = pllclk.wrapping_div(presc)
    } else {
        /* ADC34CLK clock frequency is AHB clock */
        (*RCC_Clocks).ADC34CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    }
    /* I2C1CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x10i32 as uint32_t !=
           0x10i32 as uint32_t {
        /* I2C1 Clock is HSI Osc. */
        (*RCC_Clocks).I2C1CLK_Frequency = 8000000i32 as uint32_t
    } else {
        /* I2C1 Clock is System Clock */
        (*RCC_Clocks).I2C1CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    }
    /* I2C2CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x20i32 as uint32_t !=
           0x20i32 as uint32_t {
        /* I2C2 Clock is HSI Osc. */
        (*RCC_Clocks).I2C2CLK_Frequency = 8000000i32 as uint32_t
    } else {
        /* I2C2 Clock is System Clock */
        (*RCC_Clocks).I2C2CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    }
    /* I2C3CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x40i32 as uint32_t !=
           0x40i32 as uint32_t {
        /* I2C3 Clock is HSI Osc. */
        (*RCC_Clocks).I2C3CLK_Frequency = 8000000i32 as uint32_t
    } else {
        /* I2C3 Clock is System Clock */
        (*RCC_Clocks).I2C3CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    }
    /* TIM1CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x100i32 as uint32_t ==
           0x100i32 as uint32_t && (*RCC_Clocks).SYSCLK_Frequency == pllclk &&
           apb2presc == ahbpresc {
        /* TIM1 Clock is 2 * pllclk */
        (*RCC_Clocks).TIM1CLK_Frequency =
            pllclk.wrapping_mul(2i32 as libc::c_uint)
    } else {
        /* TIM1 Clock is APB2 clock. */
        (*RCC_Clocks).TIM1CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    }
    /* TIM1CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x1000i32 as uint32_t ==
           0x1000i32 as uint32_t && (*RCC_Clocks).SYSCLK_Frequency == pllclk
           && apb2presc == ahbpresc {
        /* HRTIM1 Clock is 2 * pllclk */
        (*RCC_Clocks).HRTIM1CLK_Frequency =
            pllclk.wrapping_mul(2i32 as libc::c_uint)
    } else {
        /* HRTIM1 Clock is APB2 clock. */
        (*RCC_Clocks).HRTIM1CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    }
    /* TIM8CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x200i32 as uint32_t ==
           0x200i32 as uint32_t && (*RCC_Clocks).SYSCLK_Frequency == pllclk &&
           apb2presc == ahbpresc {
        /* TIM8 Clock is 2 * pllclk */
        (*RCC_Clocks).TIM8CLK_Frequency =
            pllclk.wrapping_mul(2i32 as libc::c_uint)
    } else {
        /* TIM8 Clock is APB2 clock. */
        (*RCC_Clocks).TIM8CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    }
    /* TIM15CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x400i32 as uint32_t ==
           0x400i32 as uint32_t && (*RCC_Clocks).SYSCLK_Frequency == pllclk &&
           apb2presc == ahbpresc {
        /* TIM15 Clock is 2 * pllclk */
        (*RCC_Clocks).TIM15CLK_Frequency =
            pllclk.wrapping_mul(2i32 as libc::c_uint)
    } else {
        /* TIM15 Clock is APB2 clock. */
        (*RCC_Clocks).TIM15CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    }
    /* TIM16CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x800i32 as uint32_t ==
           0x800i32 as uint32_t && (*RCC_Clocks).SYSCLK_Frequency == pllclk &&
           apb2presc == ahbpresc {
        /* TIM16 Clock is 2 * pllclk */
        (*RCC_Clocks).TIM16CLK_Frequency =
            pllclk.wrapping_mul(2i32 as libc::c_uint)
    } else {
        /* TIM16 Clock is APB2 clock. */
        (*RCC_Clocks).TIM16CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    }
    /* TIM17CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x2000i32 as uint32_t ==
           0x2000i32 as uint32_t && (*RCC_Clocks).SYSCLK_Frequency == pllclk
           && apb2presc == ahbpresc {
        /* TIM17 Clock is 2 * pllclk */
        (*RCC_Clocks).TIM17CLK_Frequency =
            pllclk.wrapping_mul(2i32 as libc::c_uint)
    } else {
        /* TIM17 Clock is APB2 clock. */
        (*RCC_Clocks).TIM16CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    }
    /* USART1CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x3i32 as uint32_t ==
           0i32 as libc::c_uint {
        /* USART Clock is PCLK2 */
        (*RCC_Clocks).USART1CLK_Frequency = (*RCC_Clocks).PCLK2_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x3i32 as uint32_t ==
                  0x1i32 as uint32_t {
        /* USART Clock is System Clock */
        (*RCC_Clocks).USART1CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x3i32 as uint32_t ==
                  0x2i32 as uint32_t {
        /* USART Clock is LSE Osc. */
        (*RCC_Clocks).USART1CLK_Frequency = 32768i32 as uint32_t
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x3i32 as uint32_t ==
                  0x3i32 as uint32_t {
        /* USART Clock is HSI Osc. */
        (*RCC_Clocks).USART1CLK_Frequency = 8000000i32 as uint32_t
    }
    /* USART2CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x30000i32 as uint32_t ==
           0i32 as libc::c_uint {
        /* USART Clock is PCLK */
        (*RCC_Clocks).USART2CLK_Frequency = (*RCC_Clocks).PCLK1_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x30000i32 as uint32_t ==
                  0x10000i32 as uint32_t {
        /* USART Clock is System Clock */
        (*RCC_Clocks).USART2CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x30000i32 as uint32_t ==
                  0x20000i32 as uint32_t {
        /* USART Clock is LSE Osc. */
        (*RCC_Clocks).USART2CLK_Frequency = 32768i32 as uint32_t
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x30000i32 as uint32_t ==
                  0x30000i32 as uint32_t {
        /* USART Clock is HSI Osc. */
        (*RCC_Clocks).USART2CLK_Frequency = 8000000i32 as uint32_t
    }
    /* USART3CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0xc0000i32 as uint32_t ==
           0i32 as libc::c_uint {
        /* USART Clock is PCLK */
        (*RCC_Clocks).USART3CLK_Frequency = (*RCC_Clocks).PCLK1_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0xc0000i32 as uint32_t ==
                  0x40000i32 as uint32_t {
        /* USART Clock is System Clock */
        (*RCC_Clocks).USART3CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0xc0000i32 as uint32_t ==
                  0x80000i32 as uint32_t {
        /* USART Clock is LSE Osc. */
        (*RCC_Clocks).USART3CLK_Frequency = 32768i32 as uint32_t
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0xc0000i32 as uint32_t ==
                  0xc0000i32 as uint32_t {
        /* USART Clock is HSI Osc. */
        (*RCC_Clocks).USART3CLK_Frequency = 8000000i32 as uint32_t
    }
    /* UART4CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0x300000i32 as uint32_t ==
           0i32 as libc::c_uint {
        /* USART Clock is PCLK */
        (*RCC_Clocks).UART4CLK_Frequency = (*RCC_Clocks).PCLK1_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x300000i32 as uint32_t ==
                  0x100000i32 as uint32_t {
        /* USART Clock is System Clock */
        (*RCC_Clocks).UART4CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x300000i32 as uint32_t ==
                  0x200000i32 as uint32_t {
        /* USART Clock is LSE Osc. */
        (*RCC_Clocks).UART4CLK_Frequency = 32768i32 as uint32_t
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0x300000i32 as uint32_t ==
                  0x300000i32 as uint32_t {
        /* USART Clock is HSI Osc. */
        (*RCC_Clocks).UART4CLK_Frequency = 8000000i32 as uint32_t
    }
    /* UART5CLK clock frequency */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
                                                                         as
                                                                         libc::c_uint)
              as *mut RCC_TypeDef)).CFGR3 & 0xc00000i32 as uint32_t ==
           0i32 as libc::c_uint {
        /* USART Clock is PCLK */
        (*RCC_Clocks).UART5CLK_Frequency = (*RCC_Clocks).PCLK1_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0xc00000i32 as uint32_t ==
                  0x400000i32 as uint32_t {
        /* USART Clock is System Clock */
        (*RCC_Clocks).UART5CLK_Frequency = (*RCC_Clocks).SYSCLK_Frequency
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0xc00000i32 as uint32_t ==
                  0x800000i32 as uint32_t {
        /* USART Clock is LSE Osc. */
        (*RCC_Clocks).UART5CLK_Frequency = 32768i32 as uint32_t
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x20000i32 as
                                                 libc::c_uint).wrapping_add(0x1000i32
                                                                                as
                                                                                libc::c_uint)
                     as *mut RCC_TypeDef)).CFGR3 & 0xc00000i32 as uint32_t ==
                  0xc00000i32 as uint32_t {
        /* USART Clock is HSI Osc. */
        (*RCC_Clocks).UART5CLK_Frequency = 8000000i32 as uint32_t
    };
}
/* *
  * @}
  */
/* * @defgroup RCC_Group3 Peripheral clocks configuration functions
 *  @brief   Peripheral clocks configuration functions 
 *
@verbatim   
 ===============================================================================
            ##### Peripheral clocks configuration functions #####
 ===============================================================================  
    [..] This section provide functions allowing to configure the Peripheral clocks. 
         (#) The RTC clock which is derived from the LSE, LSI or  HSE_Div32 
             (HSE divided by 32).
         (#) After restart from Reset or wakeup from STANDBY, all peripherals are 
             off except internal SRAM, Flash and SWD. Before to start using 
             a peripheral you have to enable its interface clock. You can do this 
             using RCC_AHBPeriphClockCmd(), RCC_APB2PeriphClockCmd() 
             and RCC_APB1PeriphClockCmd() functions.
         (#) To reset the peripherals configuration (to the default state after 
             device reset) you can use RCC_AHBPeriphResetCmd(), RCC_APB2PeriphResetCmd() 
             and RCC_APB1PeriphResetCmd() functions.
@endverbatim
  * @{
  */
/* *
  * @brief  Configures the ADC clock (ADCCLK).
  * @param  RCC_PLLCLK: defines the ADC clock divider. This clock is derived from 
  *         the PLL Clock.
  *   This parameter can be one of the following values:
  *     @arg RCC_ADC12PLLCLK_OFF: ADC12 clock disabled
  *     @arg RCC_ADC12PLLCLK_Div1: ADC12 clock = PLLCLK/1
  *     @arg RCC_ADC12PLLCLK_Div2: ADC12 clock = PLLCLK/2
  *     @arg RCC_ADC12PLLCLK_Div4: ADC12 clock = PLLCLK/4
  *     @arg RCC_ADC12PLLCLK_Div6: ADC12 clock = PLLCLK/6
  *     @arg RCC_ADC12PLLCLK_Div8: ADC12 clock = PLLCLK/8
  *     @arg RCC_ADC12PLLCLK_Div10: ADC12 clock = PLLCLK/10
  *     @arg RCC_ADC12PLLCLK_Div12: ADC12 clock = PLLCLK/12
  *     @arg RCC_ADC12PLLCLK_Div16: ADC12 clock = PLLCLK/16
  *     @arg RCC_ADC12PLLCLK_Div32: ADC12 clock = PLLCLK/32
  *     @arg RCC_ADC12PLLCLK_Div64: ADC12 clock = PLLCLK/64
  *     @arg RCC_ADC12PLLCLK_Div128: ADC12 clock = PLLCLK/128
  *     @arg RCC_ADC12PLLCLK_Div256: ADC12 clock = PLLCLK/256
  *     @arg RCC_ADC34PLLCLK_OFF: ADC34 clock disabled
  *     @arg RCC_ADC34PLLCLK_Div1: ADC34 clock = PLLCLK/1
  *     @arg RCC_ADC34PLLCLK_Div2: ADC34 clock = PLLCLK/2
  *     @arg RCC_ADC34PLLCLK_Div4: ADC34 clock = PLLCLK/4
  *     @arg RCC_ADC34PLLCLK_Div6: ADC34 clock = PLLCLK/6
  *     @arg RCC_ADC34PLLCLK_Div8: ADC34 clock = PLLCLK/8
  *     @arg RCC_ADC34PLLCLK_Div10: ADC34 clock = PLLCLK/10
  *     @arg RCC_ADC34PLLCLK_Div12: ADC34 clock = PLLCLK/12
  *     @arg RCC_ADC34PLLCLK_Div16: ADC34 clock = PLLCLK/16
  *     @arg RCC_ADC34PLLCLK_Div32: ADC34 clock = PLLCLK/32
  *     @arg RCC_ADC34PLLCLK_Div64: ADC34 clock = PLLCLK/64       
  *     @arg RCC_ADC34PLLCLK_Div128: ADC34 clock = PLLCLK/128                                  
  *     @arg RCC_ADC34PLLCLK_Div256: ADC34 clock = PLLCLK/256
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ADCCLKConfig(mut RCC_PLLCLK: uint32_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp = RCC_PLLCLK >> 28i32;
    /* Clears ADCPRE34 bits */
    if tmp != 0i32 as libc::c_uint {
        let ref mut fresh14 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR2;
        ::core::ptr::write_volatile(fresh14,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x3e00i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    } else {
        /* Clears ADCPRE12 bits */
        let ref mut fresh15 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR2;
        ::core::ptr::write_volatile(fresh15,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1f0i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    }
    /* Set ADCPRE bits according to RCC_PLLCLK value */
    let ref mut fresh16 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR2;
    ::core::ptr::write_volatile(fresh16,
                                (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_PLLCLK) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Configures the I2C clock (I2CCLK).
  * @param  RCC_I2CCLK: defines the I2C clock source. This clock is derived 
  *         from the HSI or System clock.
  *   This parameter can be one of the following values:
  *     @arg RCC_I2CxCLK_HSI: I2Cx clock = HSI
  *     @arg RCC_I2CxCLK_SYSCLK: I2Cx clock = System Clock
  *          (x can be 1 or 2 or 3).  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_I2CCLKConfig(mut RCC_I2CCLK: uint32_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp = RCC_I2CCLK >> 28i32;
    /* Clear I2CSW bit */
    match tmp {
        0 => {
            let ref mut fresh17 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh17,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        1 => {
            let ref mut fresh18 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh18,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        2 => {
            let ref mut fresh19 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh19,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x40i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Set I2CSW bits according to RCC_I2CCLK value */
    let ref mut fresh20 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR3;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_I2CCLK) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Configures the TIMx clock sources(TIMCLK).
  * @note     The configuration of the TIMx clock source is only possible when the 
  *           SYSCLK = PLL and HCLK and PCLK2 clocks are not divided in respect to SYSCLK
  * @note     If one of the previous conditions is missed, the TIM clock source 
  *           configuration is lost and calling again this function becomes mandatory.  
  * @param  RCC_TIMCLK: defines the TIMx clock source.
  *   This parameter can be one of the following values:
  *     @arg RCC_TIMxCLK_HCLK: TIMx clock = APB high speed clock (doubled frequency
  *          when prescaled)
  *     @arg RCC_TIMxCLK_PLLCLK: TIMx clock = PLL output (running up to 144 MHz)
  *          (x can be 1, 8, 15, 16, 17).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_TIMCLKConfig(mut RCC_TIMCLK: uint32_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp = RCC_TIMCLK >> 28i32;
    /* Clear TIMSW bit */
    match tmp {
        0 => {
            let ref mut fresh21 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh21,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        1 => {
            let ref mut fresh22 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh22,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x200i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        2 => {
            let ref mut fresh23 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh23,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x400i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        3 => {
            let ref mut fresh24 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh24,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x800i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            let ref mut fresh25 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh25,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x2000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Set I2CSW bits according to RCC_TIMCLK value */
    let ref mut fresh26 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR3;
    ::core::ptr::write_volatile(fresh26,
                                (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_TIMCLK) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Configures the HRTIM1 clock sources(HRTIM1CLK).
  * @note     The configuration of the HRTIM1 clock source is only possible when the 
  *           SYSCLK = PLL and HCLK and PCLK2 clocks are not divided in respect to SYSCLK
  * @note     If one of the previous conditions is missed, the TIM clock source 
  *           configuration is lost and calling again this function becomes mandatory.  
  * @param  RCC_HRTIMCLK: defines the TIMx clock source.
  *   This parameter can be one of the following values:
  *     @arg RCC_HRTIM1CLK_HCLK: TIMx clock = APB high speed clock (doubled frequency
  *          when prescaled)
  *     @arg RCC_HRTIM1CLK_PLLCLK: TIMx clock = PLL output (running up to 144 MHz)
  *          (x can be 1 or 8).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_HRTIM1CLKConfig(mut RCC_HRTIMCLK: uint32_t) {
    /* Check the parameters */
    /* Clear HRTIMSW bit */
    let ref mut fresh27 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR3;
    ::core::ptr::write_volatile(fresh27,
                                (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Set HRTIMSW bits according to RCC_HRTIMCLK value */
    let ref mut fresh28 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR3;
    ::core::ptr::write_volatile(fresh28,
                                (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_HRTIMCLK) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the USART clock (USARTCLK).
  * @param  RCC_USARTCLK: defines the USART clock source. This clock is derived 
  *         from the HSI or System clock.
  *   This parameter can be one of the following values:
  *     @arg RCC_USARTxCLK_PCLK: USART clock = APB Clock (PCLK)
  *     @arg RCC_USARTxCLK_SYSCLK: USART clock = System Clock
  *     @arg RCC_USARTxCLK_LSE: USART clock = LSE Clock
  *     @arg RCC_USARTxCLK_HSI: USART clock = HSI Clock
  *          (x can be 1, 2, 3, 4 or 5).  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_USARTCLKConfig(mut RCC_USARTCLK: uint32_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp = RCC_USARTCLK >> 28i32;
    /* Clear USARTSW[1:0] bit */
    match tmp {
        1 => {
            /* clear USART1SW */
            let ref mut fresh29 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh29,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x3i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        2 => {
            /* clear USART2SW */
            let ref mut fresh30 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh30,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh30
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x30000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        3 => {
            /* clear USART3SW */
            let ref mut fresh31 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh31,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh31
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0xc0000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        4 => {
            /* clear UART4SW */
            let ref mut fresh32 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh32,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh32
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x300000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        5 => {
            /* clear UART5SW */
            let ref mut fresh33 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x20000i32 as
                                                   libc::c_uint).wrapping_add(0x1000i32
                                                                                  as
                                                                                  libc::c_uint)
                       as *mut RCC_TypeDef)).CFGR3;
            ::core::ptr::write_volatile(fresh33,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh33
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0xc00000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    }
    /* Set USARTSW bits according to RCC_USARTCLK value */
    let ref mut fresh34 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CFGR3;
    ::core::ptr::write_volatile(fresh34,
                                (::core::ptr::read_volatile::<uint32_t>(fresh34
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_USARTCLK) as
                                    uint32_t as uint32_t);
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
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x4i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x16i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, RCC_USBCLKSource);
}
/* *
  * @brief  Configures the RTC clock (RTCCLK).
  * @note     As the RTC clock configuration bits are in the Backup domain and write
  *           access is denied to this domain after reset, you have to enable write
  *           access using PWR_BackupAccessCmd(ENABLE) function before to configure
  *           the RTC clock source (to be done once after reset).    
  * @note     Once the RTC clock is configured it can't be changed unless the RTC
  *           is reset using RCC_BackupResetCmd function, or by a Power On Reset (POR)
  *             
  * @param  RCC_RTCCLKSource: specifies the RTC clock source.
  *   This parameter can be one of the following values:
  *     @arg RCC_RTCCLKSource_LSE: LSE selected as RTC clock
  *     @arg RCC_RTCCLKSource_LSI: LSI selected as RTC clock
  *     @arg RCC_RTCCLKSource_HSE_Div32: HSE divided by 32 selected as RTC clock
  *       
  * @note     If the LSE or LSI is used as RTC clock source, the RTC continues to
  *           work in STOP and STANDBY modes, and can be used as wakeup source.
  *           However, when the HSE clock is used as RTC clock source, the RTC
  *           cannot be used in STOP and STANDBY modes.             
  * @note     The maximum input clock frequency for RTC is 2MHz (when using HSE as
  *           RTC clock source).             
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_RTCCLKConfig(mut RCC_RTCCLKSource: uint32_t) {
    /* Check the parameters */
    /* Select the RTC clock source */
    let ref mut fresh35 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).BDCR;
    ::core::ptr::write_volatile(fresh35,
                                (::core::ptr::read_volatile::<uint32_t>(fresh35
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RCC_RTCCLKSource) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the I2S clock source (I2SCLK).
  * @note   This function must be called before enabling the SPI2 and SPI3 clocks.
  * @param  RCC_I2SCLKSource: specifies the I2S clock source.
  *          This parameter can be one of the following values:
  *            @arg RCC_I2S2CLKSource_SYSCLK: SYSCLK clock used as I2S clock source
  *            @arg RCC_I2S2CLKSource_Ext: External clock mapped on the I2S_CKIN pin
  *                                        used as I2S clock source
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_I2SCLKConfig(mut RCC_I2SCLKSource: uint32_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x4i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x17i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, RCC_I2SCLKSource);
}
/* *
  * @brief  Enables or disables the RTC clock.
  * @note   This function must be used only after the RTC clock source was selected
  *         using the RCC_RTCCLKConfig function.
  * @param  NewState: new state of the RTC clock.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_RTCCLKCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x20i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0xfi32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Forces or releases the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCC_BDCR register.
  * @param  NewState: new state of the Backup domain reset.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_BackupResetCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    ::core::ptr::write_volatile((0x42000000i32 as
                                     uint32_t).wrapping_add((0x40000000i32 as
                                                                 uint32_t).wrapping_add(0x20000i32
                                                                                            as
                                                                                            libc::c_uint).wrapping_add(0x1000i32
                                                                                                                           as
                                                                                                                           libc::c_uint).wrapping_sub(0x40000000i32
                                                                                                                                                          as
                                                                                                                                                          uint32_t).wrapping_add(0x20i32
                                                                                                                                                                                     as
                                                                                                                                                                                     libc::c_uint).wrapping_mul(32i32
                                                                                                                                                                                                                    as
                                                                                                                                                                                                                    libc::c_uint)).wrapping_add((0x10i32
                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                     4i32)
                                                                                                                                                                                                                                                    as
                                                                                                                                                                                                                                                    libc::c_uint)
                                    as *mut uint32_t, NewState as uint32_t);
}
/* *
  * @brief  Enables or disables the AHB peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.    
  * @param  RCC_AHBPeriph: specifies the AHB peripheral to gates its clock.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_AHBPeriph_GPIOA
  *     @arg RCC_AHBPeriph_GPIOB
  *     @arg RCC_AHBPeriph_GPIOC  
  *     @arg RCC_AHBPeriph_GPIOD
  *     @arg RCC_AHBPeriph_GPIOE  
  *     @arg RCC_AHBPeriph_GPIOF
  *     @arg RCC_AHBPeriph_TS
  *     @arg RCC_AHBPeriph_CRC
  *     @arg RCC_AHBPeriph_FLITF (has effect only when the Flash memory is in power down mode)  
  *     @arg RCC_AHBPeriph_SRAM
  *     @arg RCC_AHBPeriph_DMA2
  *     @arg RCC_AHBPeriph_DMA1
  *     @arg RCC_AHBPeriph_ADC34
  *     @arg RCC_AHBPeriph_ADC12      
  * @param  NewState: new state of the specified peripheral clock.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_AHBPeriphClockCmd(mut RCC_AHBPeriph: uint32_t,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh36 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).AHBENR;
        ::core::ptr::write_volatile(fresh36,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh36
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_AHBPeriph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh37 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).AHBENR;
        ::core::ptr::write_volatile(fresh37,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh37
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_AHBPeriph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the High Speed APB (APB2) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to gates its clock.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB2Periph_SYSCFG
  *     @arg RCC_APB2Periph_SPI1
  *     @arg RCC_APB2Periph_USART1
  *     @arg RCC_APB2Periph_TIM15
  *     @arg RCC_APB2Periph_TIM16
  *     @arg RCC_APB2Periph_TIM17
  *     @arg RCC_APB2Periph_TIM1       
  *     @arg RCC_APB2Periph_TIM8
  *     @arg RCC_APB2Periph_HRTIM1  
  * @param  NewState: new state of the specified peripheral clock.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB2PeriphClockCmd(mut RCC_APB2Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh38 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh38,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh38
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh39 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2ENR;
        ::core::ptr::write_volatile(fresh39,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh39
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the Low Speed APB (APB1) peripheral clock.
  * @note   After reset, the peripheral clock (used for registers read/write access)
  *         is disabled and the application software has to enable this clock before 
  *         using it.
  * @param  RCC_APB1Periph: specifies the APB1 peripheral to gates its clock.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB1Periph_TIM2
  *     @arg RCC_APB1Periph_TIM3
  *     @arg RCC_APB1Periph_TIM4
  *     @arg RCC_APB1Periph_TIM6
  *     @arg RCC_APB1Periph_TIM7
  *     @arg RCC_APB1Periph_WWDG
  *     @arg RCC_APB1Periph_SPI2
  *     @arg RCC_APB1Periph_SPI3  
  *     @arg RCC_APB1Periph_USART2
  *     @arg RCC_APB1Periph_USART3
  *     @arg RCC_APB1Periph_UART4 
  *     @arg RCC_APB1Periph_UART5     
  *     @arg RCC_APB1Periph_I2C1
  *     @arg RCC_APB1Periph_I2C2
  *     @arg RCC_APB1Periph_USB
  *     @arg RCC_APB1Periph_CAN1
  *     @arg RCC_APB1Periph_PWR
  *     @arg RCC_APB1Periph_DAC1
  *     @arg RCC_APB1Periph_DAC2  
  * @param  NewState: new state of the specified peripheral clock.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB1PeriphClockCmd(mut RCC_APB1Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh40 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1ENR;
        ::core::ptr::write_volatile(fresh40,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh40
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh41 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1ENR;
        ::core::ptr::write_volatile(fresh41,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh41
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Forces or releases AHB peripheral reset.
  * @param  RCC_AHBPeriph: specifies the AHB peripheral to reset.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_AHBPeriph_GPIOA
  *     @arg RCC_AHBPeriph_GPIOB
  *     @arg RCC_AHBPeriph_GPIOC  
  *     @arg RCC_AHBPeriph_GPIOD
  *     @arg RCC_AHBPeriph_GPIOE  
  *     @arg RCC_AHBPeriph_GPIOF
  *     @arg RCC_AHBPeriph_TS
  *     @arg RCC_AHBPeriph_ADC34
  *     @arg RCC_AHBPeriph_ADC12    
  * @param  NewState: new state of the specified peripheral reset.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_AHBPeriphResetCmd(mut RCC_AHBPeriph: uint32_t,
                                               mut NewState:
                                                   FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh42 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).AHBRSTR;
        ::core::ptr::write_volatile(fresh42,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh42
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_AHBPeriph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh43 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).AHBRSTR;
        ::core::ptr::write_volatile(fresh43,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh43
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_AHBPeriph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Forces or releases High Speed APB (APB2) peripheral reset.
  * @param  RCC_APB2Periph: specifies the APB2 peripheral to reset.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_APB2Periph_SYSCFG
  *     @arg RCC_APB2Periph_SPI1
  *     @arg RCC_APB2Periph_USART1
  *     @arg RCC_APB2Periph_TIM15
  *     @arg RCC_APB2Periph_TIM16
  *     @arg RCC_APB2Periph_TIM17
  *     @arg RCC_APB2Periph_TIM1       
  *     @arg RCC_APB2Periph_TIM8 
  *     @arg RCC_APB2Periph_HRTIM1       
  * @param  NewState: new state of the specified peripheral reset.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB2PeriphResetCmd(mut RCC_APB2Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh44 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2RSTR;
        ::core::ptr::write_volatile(fresh44,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh44
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB2Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh45 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB2RSTR;
        ::core::ptr::write_volatile(fresh45,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh45
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
  *     @arg RCC_APB1Periph_TIM2
  *     @arg RCC_APB1Periph_TIM3
  *     @arg RCC_APB1Periph_TIM4
  *     @arg RCC_APB1Periph_TIM6
  *     @arg RCC_APB1Periph_TIM7
  *     @arg RCC_APB1Periph_WWDG
  *     @arg RCC_APB1Periph_SPI2
  *     @arg RCC_APB1Periph_SPI3  
  *     @arg RCC_APB1Periph_USART2
  *     @arg RCC_APB1Periph_USART3
  *     @arg RCC_APB1Periph_UART4
  *     @arg RCC_APB1Periph_UART5      
  *     @arg RCC_APB1Periph_I2C1
  *     @arg RCC_APB1Periph_I2C2
  *     @arg RCC_APB1Periph_I2C3
  *     @arg RCC_APB1Periph_USB
  *     @arg RCC_APB1Periph_CAN1
  *     @arg RCC_APB1Periph_PWR
  *     @arg RCC_APB1Periph_DAC
  * @param  NewState: new state of the specified peripheral clock.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_APB1PeriphResetCmd(mut RCC_APB1Periph: uint32_t,
                                                mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh46 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1RSTR;
        ::core::ptr::write_volatile(fresh46,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh46
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh47 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).APB1RSTR;
        ::core::ptr::write_volatile(fresh47,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh47
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RCC_APB1Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup RCC_Group4 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions 
 *
@verbatim   
 ===============================================================================
            ##### Interrupts and flags management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified RCC interrupts.
  * @note   The CSS interrupt doesn't have an enable bit; once the CSS is enabled
  *         and if the HSE clock fails, the CSS interrupt occurs and an NMI is
  *         automatically generated. The NMI will be executed indefinitely, and 
  *         since NMI has higher priority than any other IRQ (and main program)
  *         the application will be stacked in the NMI ISR unless the CSS interrupt
  *         pending bit is cleared.
  * @param  RCC_IT: specifies the RCC interrupt sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  * @param  NewState: new state of the specified RCC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ITConfig(mut RCC_IT: uint8_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Perform Byte access to RCC_CIR[13:8] bits to enable the selected interrupts */
        let ref mut fresh48 = *(0x40021009i32 as uint32_t as *mut uint8_t);
        ::core::ptr::write_volatile(fresh48,
                                    (::core::ptr::read_volatile::<uint8_t>(fresh48
                                                                               as
                                                                               *const uint8_t)
                                         as libc::c_int |
                                         RCC_IT as libc::c_int) as uint8_t as
                                        uint8_t)
    } else {
        /* Perform Byte access to RCC_CIR[13:8] bits to disable the selected interrupts */
        let ref mut fresh49 = *(0x40021009i32 as uint32_t as *mut uint8_t);
        ::core::ptr::write_volatile(fresh49,
                                    (::core::ptr::read_volatile::<uint8_t>(fresh49
                                                                               as
                                                                               *const uint8_t)
                                         as libc::c_int &
                                         !(RCC_IT as libc::c_int) as uint8_t
                                             as libc::c_int) as uint8_t as
                                        uint8_t)
    };
}
/* *
  * @brief  Checks whether the specified RCC flag is set or not.
  * @param  RCC_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg RCC_FLAG_HSIRDY: HSI oscillator clock ready  
  *     @arg RCC_FLAG_HSERDY: HSE oscillator clock ready
  *     @arg RCC_FLAG_PLLRDY: PLL clock ready
  *     @arg RCC_FLAG_MCOF: MCO Flag  
  *     @arg RCC_FLAG_LSERDY: LSE oscillator clock ready
  *     @arg RCC_FLAG_LSIRDY: LSI oscillator clock ready
  *     @arg RCC_FLAG_OBLRST: Option Byte Loader (OBL) reset 
  *     @arg RCC_FLAG_PINRST: Pin reset
  *     @arg RCC_FLAG_PORRST: POR/PDR reset
  *     @arg RCC_FLAG_SFTRST: Software reset
  *     @arg RCC_FLAG_IWDGRST: Independent Watchdog reset
  *     @arg RCC_FLAG_WWDGRST: Window Watchdog reset
  *     @arg RCC_FLAG_LPWRRST: Low Power reset
  * @retval The new state of RCC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetFlagStatus(mut RCC_FLAG: uint8_t)
 -> FlagStatus {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    let mut statusreg: uint32_t = 0i32 as uint32_t;
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    /* Get the RCC register index */
    tmp = (RCC_FLAG as libc::c_int >> 5i32) as uint32_t;
    if tmp == 0i32 as libc::c_uint {
        /* The flag to check is in CR register */
        statusreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CR
    } else if tmp == 1i32 as libc::c_uint {
        /* The flag to check is in BDCR register */
        statusreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).BDCR
    } else if tmp == 4i32 as libc::c_uint {
        /* The flag to check is in CFGR register */
        statusreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CFGR
    } else {
        /* The flag to check is in CSR register */
        statusreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x20000i32 as
                                               libc::c_uint).wrapping_add(0x1000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut RCC_TypeDef)).CSR
    }
    /* Get the flag position */
    tmp =
        (RCC_FLAG as libc::c_int & 0x1fi32 as uint8_t as libc::c_int) as
            uint32_t;
    if statusreg & (1i32 as uint32_t) << tmp !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the flag status */
    return bitstatus;
}
/* *
  * @brief  Clears the RCC reset flags.
  *         The reset flags are: RCC_FLAG_OBLRST, RCC_FLAG_PINRST, RCC_FLAG_PORRST, 
  *         RCC_FLAG_SFTRST, RCC_FLAG_IWDGRST, RCC_FLAG_WWDGRST, RCC_FLAG_LPWRRST.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ClearFlag() {
    /* Set RMVF bit to clear the reset flags */
    let ref mut fresh50 =
        (*((0x40000000i32 as
                uint32_t).wrapping_add(0x20000i32 as
                                           libc::c_uint).wrapping_add(0x1000i32
                                                                          as
                                                                          libc::c_uint)
               as *mut RCC_TypeDef)).CSR;
    ::core::ptr::write_volatile(fresh50,
                                (::core::ptr::read_volatile::<uint32_t>(fresh50
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x1000000i32 as uint32_t) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Checks whether the specified RCC interrupt has occurred or not.
  * @param  RCC_IT: specifies the RCC interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *     @arg RCC_IT_CSS: Clock Security System interrupt
  * @retval The new state of RCC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_GetITStatus(mut RCC_IT: uint8_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    /* Check the status of the specified RCC interrupt */
    if (*((0x40000000i32 as
               uint32_t).wrapping_add(0x20000i32 as
                                          libc::c_uint).wrapping_add(0x1000i32
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
  * @file    stm32f30x_rcc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the RCC 
  *          firmware library.
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
/* * @addtogroup RCC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup RCC_Exported_Constants
  * @{
  */
/* * @defgroup RCC_HSE_configuration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLL_Clock_Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_PLL_Multiplication_Factor 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_PREDIV1_division_factor
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_System_Clock_Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_AHB_Clock_Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_APB2_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_ADC_clock_source 
  * @{
  */
/* ADC1 & ADC2 */
/* ADC3 & ADC4 */
/* *
  * @}
  */
/* * @defgroup RCC_TIM_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_HRTIM_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_I2C_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_USART_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Interrupt_Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_LSE_configuration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_RTC_Clock_Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_I2S_Clock_Source
  * @{
  */
/* * @defgroup RCC_LSE_Drive_Configuration 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_AHB_Peripherals 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB2_Peripherals 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_APB1_Peripherals 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_MCO_Clock_Source
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_MCOPrescaler
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_USB_Device_clock_source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RCC_Flag 
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
/* Function used to set the RCC clock configuration to the default reset state */
/* Internal/external clocks, PLL, CSS and MCO configuration functions *********/
/* STM32F303xC */
/* System, AHB and APB busses clocks configuration functions ******************/
/* Peripheral clocks configuration functions **********************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the RCC's interrupt pending bits.
  * @param  RCC_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RCC_IT_LSIRDY: LSI ready interrupt
  *     @arg RCC_IT_LSERDY: LSE ready interrupt
  *     @arg RCC_IT_HSIRDY: HSI ready interrupt
  *     @arg RCC_IT_HSERDY: HSE ready interrupt
  *     @arg RCC_IT_PLLRDY: PLL ready interrupt
  *     @arg RCC_IT_CSS: Clock Security System interrupt
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RCC_ClearITPendingBit(mut RCC_IT: uint8_t) {
    /* Check the parameters */
    /* Perform Byte access to RCC_CIR[23:16] bits to clear the selected interrupt
     pending bits */
    ::core::ptr::write_volatile(0x4002100ai32 as uint32_t as *mut uint8_t,
                                RCC_IT);
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
