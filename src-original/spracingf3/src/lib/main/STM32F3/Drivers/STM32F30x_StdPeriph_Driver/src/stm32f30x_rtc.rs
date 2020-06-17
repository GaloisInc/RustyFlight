use core;
use libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
  * @brief Real-Time Clock
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RTC_TypeDef {
    pub TR: uint32_t,
    pub DR: uint32_t,
    pub CR: uint32_t,
    pub ISR: uint32_t,
    pub PRER: uint32_t,
    pub WUTR: uint32_t,
    pub RESERVED0: uint32_t,
    pub ALRMAR: uint32_t,
    pub ALRMBR: uint32_t,
    pub WPR: uint32_t,
    pub SSR: uint32_t,
    pub SHIFTR: uint32_t,
    pub TSTR: uint32_t,
    pub TSDR: uint32_t,
    pub TSSSR: uint32_t,
    pub CALR: uint32_t,
    pub TAFCR: uint32_t,
    pub ALRMASSR: uint32_t,
    pub ALRMBSSR: uint32_t,
    pub RESERVED7: uint32_t,
    pub BKP0R: uint32_t,
    pub BKP1R: uint32_t,
    pub BKP2R: uint32_t,
    pub BKP3R: uint32_t,
    pub BKP4R: uint32_t,
    pub BKP5R: uint32_t,
    pub BKP6R: uint32_t,
    pub BKP7R: uint32_t,
    pub BKP8R: uint32_t,
    pub BKP9R: uint32_t,
    pub BKP10R: uint32_t,
    pub BKP11R: uint32_t,
    pub BKP12R: uint32_t,
    pub BKP13R: uint32_t,
    pub BKP14R: uint32_t,
    pub BKP15R: uint32_t,
    /* !< RTC backup register 15,                                   Address offset: 0x8C */
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RTC_InitTypeDef {
    pub RTC_HourFormat: uint32_t,
    pub RTC_AsynchPrediv: uint32_t,
    pub RTC_SynchPrediv: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RTC_TimeTypeDef {
    pub RTC_Hours: uint8_t,
    pub RTC_Minutes: uint8_t,
    pub RTC_Seconds: uint8_t,
    pub RTC_H12: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RTC_DateTypeDef {
    pub RTC_WeekDay: uint8_t,
    pub RTC_Month: uint8_t,
    pub RTC_Date: uint8_t,
    pub RTC_Year: uint8_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct RTC_AlarmTypeDef {
    pub RTC_AlarmTime: RTC_TimeTypeDef,
    pub RTC_AlarmMask: uint32_t,
    pub RTC_AlarmDateWeekDaySel: uint32_t,
    pub RTC_AlarmDateWeekDay: uint8_t,
}
/* Private functions ---------------------------------------------------------*/
/* * @defgroup RTC_Private_Functions
  * @{
  */
/* * @defgroup RTC_Group1 Initialization and Configuration functions
 *  @brief   Initialization and Configuration functions 
 *
@verbatim   
 ===============================================================================
            ##### Initialization and Configuration functions #####
 ===============================================================================  
    [..] This section provide functions allowing to initialize and configure the RTC
         Prescaler (Synchronous and Asynchronous), RTC Hour format, disable RTC registers
         Write protection, enter and exit the RTC initialization mode, RTC registers
         synchronization check and reference clock detection enable.
         (#) The RTC Prescaler is programmed to generate the RTC 1Hz time base. It is
             split into 2 programmable prescalers to minimize power consumption.
             (++) A 7-bit asynchronous prescaler and A 13-bit synchronous prescaler.
             (++) When both prescalers are used, it is recommended to configure the 
                  asynchronous prescaler to a high value to minimize consumption.
         (#) All RTC registers are Write protected. Writing to the RTC registers
             is enabled by writing a key into the Write Protection register, RTC_WPR.
         (#) To Configure the RTC Calendar, user application should enter initialization
             mode. In this mode, the calendar counter is stopped and its value 
             can be updated. When the initialization sequence is complete, the 
             calendar restarts counting after 4 RTCCLK cycles.
         (#) To read the calendar through the shadow registers after Calendar 
             initialization, calendar update or after wakeup from low power modes 
             the software must first clear the RSF flag. The software must then 
             wait until it is set again before reading the calendar, which means 
             that the calendar registers have been correctly copied into the RTC_TR 
             and RTC_DR shadow registers. The RTC_WaitForSynchro() function 
             implements the above software sequence (RSF clear and RSF check).

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the RTC registers to their default reset values.
  * @note   This function doesn't reset the RTC Clock source and RTC Backup Data
  *         registers.       
  * @param  None
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are deinitialized
  *          - ERROR: RTC registers are not deinitialized
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_DeInit() -> ErrorStatus {
    let mut wutcounter: uint32_t = 0i32 as uint32_t;
    let mut wutwfstatus: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Set Initialization mode */
    if RTC_EnterInitMode() as libc::c_uint ==
           ERROR as libc::c_int as libc::c_uint {
        status = ERROR
    } else {
        /* Reset TR, DR and CR registers */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).TR as
                                        *mut uint32_t, 0i32 as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).DR as
                                        *mut uint32_t, 0x2101i32 as uint32_t);
        /* Reset All CR bits except CR[2:0] */
        let ref mut fresh0 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & 0x7i32 as uint32_t)
                                        as uint32_t as uint32_t);
        loop 
             /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
             {
            wutwfstatus =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).ISR & 0x4i32 as uint32_t;
            ::core::ptr::write_volatile(&mut wutcounter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&wutcounter
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if !(wutcounter != 0x2000i32 as uint32_t &&
                     wutwfstatus == 0i32 as libc::c_uint) {
                break ;
            }
        }
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).ISR & 0x4i32 as uint32_t ==
               RESET as libc::c_int as libc::c_uint {
            status = ERROR
        } else {
            /* Reset all RTC CR register bits */
            let ref mut fresh1 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh1,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             0i32 as uint32_t) as uint32_t as
                                            uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut RTC_TypeDef)).WUTR
                                            as *mut uint32_t,
                                        0xffffi32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut RTC_TypeDef)).PRER
                                            as *mut uint32_t,
                                        0x7f00ffi32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).ALRMAR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).ALRMBR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).SHIFTR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut RTC_TypeDef)).CALR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).ALRMASSR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).ALRMBSSR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            /* Reset ISR register and exit initialization mode */
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as *mut RTC_TypeDef)).ISR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            /* Reset Tamper and alternate functions configuration register */
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).TAFCR
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            /* Wait till the RTC RSF flag is set */
            if RTC_WaitForSynchro() as libc::c_uint ==
                   ERROR as libc::c_int as libc::c_uint {
                status = ERROR
            } else { status = SUCCESS }
        }
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @brief  Initializes the RTC registers according to the specified parameters 
  *         in RTC_InitStruct.
  * @param  RTC_InitStruct: pointer to a RTC_InitTypeDef structure that contains 
  *         the configuration information for the RTC peripheral.
  * @note   The RTC Prescaler register is write protected and can be written in 
  *         initialization mode only.  
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are initialized
  *          - ERROR: RTC registers are not initialized  
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_Init(mut RTC_InitStruct: *mut RTC_InitTypeDef)
 -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Set Initialization mode */
    if RTC_EnterInitMode() as libc::c_uint ==
           ERROR as libc::c_int as libc::c_uint {
        status = ERROR
    } else {
        /* Clear RTC CR FMT Bit */
        let ref mut fresh2 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x40i32 as uint32_t)) as uint32_t
                                        as uint32_t);
        /* Set RTC_CR register */
        let ref mut fresh3 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*RTC_InitStruct).RTC_HourFormat) as
                                        uint32_t as uint32_t);
        /* Configure the RTC PRER */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).PRER as
                                        *mut uint32_t,
                                    (*RTC_InitStruct).RTC_SynchPrediv);
        let ref mut fresh4 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).PRER;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         (*RTC_InitStruct).RTC_AsynchPrediv <<
                                             16i32) as uint32_t as uint32_t);
        /* Exit Initialization mode */
        RTC_ExitInitMode();
        status = SUCCESS
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @brief  Fills each RTC_InitStruct member with its default value.
  * @param  RTC_InitStruct: pointer to a RTC_InitTypeDef structure which will be 
  *         initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_StructInit(mut RTC_InitStruct:
                                            *mut RTC_InitTypeDef) {
    /* Initialize the RTC_HourFormat member */
    (*RTC_InitStruct).RTC_HourFormat = 0i32 as uint32_t;
    /* Initialize the RTC_AsynchPrediv member */
    (*RTC_InitStruct).RTC_AsynchPrediv = 0x7fi32 as uint32_t;
    /* Initialize the RTC_SynchPrediv member */
    (*RTC_InitStruct).RTC_SynchPrediv = 0xffi32 as uint32_t;
}
/* *
  * @brief  Enables or disables the RTC registers write protection.
  * @note   All the RTC registers are write protected except for RTC_ISR[13:8], 
  *         RTC_TAFCR and RTC_BKPxR.
  * @note   Writing a wrong key reactivates the write protection.
  * @note   The protection mechanism is not affected by system reset.  
  * @param  NewState: new state of the write protection.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WriteProtectionCmd(mut NewState:
                                                    FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the write protection for RTC registers */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).WPR as
                                        *mut uint32_t, 0xffi32 as uint32_t)
    } else {
        /* Disable the write protection for RTC registers */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).WPR as
                                        *mut uint32_t, 0xcai32 as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).WPR as
                                        *mut uint32_t, 0x53i32 as uint32_t)
    };
}
/* *
  * @brief  Enters the RTC Initialization mode.
  * @note   The RTC Initialization mode is write protected, use the 
  *         RTC_WriteProtectionCmd(DISABLE) before calling this function.    
  * @param  None
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC is in Init mode
  *          - ERROR: RTC is not in Init mode  
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_EnterInitMode() -> ErrorStatus {
    let mut initcounter: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    let mut initstatus: uint32_t = 0i32 as uint32_t;
    /* Check if the Initialization mode is set */
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
              as *mut RTC_TypeDef)).ISR & 0x40i32 as uint32_t ==
           RESET as libc::c_int as uint32_t {
        /* Set the Initialization mode */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).ISR as
                                        *mut uint32_t, 0xffffffffu32);
        loop 
             /* Wait till RTC is in INIT state and if Time out is reached exit */
             {
            initstatus =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).ISR & 0x40i32 as uint32_t;
            ::core::ptr::write_volatile(&mut initcounter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&initcounter
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if !(initcounter != 0x2000i32 as uint32_t &&
                     initstatus == 0i32 as libc::c_uint) {
                break ;
            }
        }
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).ISR & 0x40i32 as uint32_t !=
               RESET as libc::c_int as libc::c_uint {
            status = SUCCESS
        } else { status = ERROR }
    } else { status = SUCCESS }
    return status;
}
/* *
  * @brief  Exits the RTC Initialization mode.
  * @note   When the initialization sequence is complete, the calendar restarts 
  *         counting after 4 RTCCLK cycles.  
  * @note   The RTC Initialization mode is write protected, use the 
  *         RTC_WriteProtectionCmd(DISABLE) before calling this function.      
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ExitInitMode() {
    /* Exit Initialization mode */
    let ref mut fresh5 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).ISR;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x80i32 as uint32_t))
                                    as uint32_t as uint32_t);
}
/* *
  * @brief  Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are 
  *         synchronized with RTC APB clock.
  * @note   The RTC Resynchronization mode is write protected, use the 
  *         RTC_WriteProtectionCmd(DISABLE) before calling this function. 
  * @note   To read the calendar through the shadow registers after Calendar 
  *         initialization, calendar update or after wakeup from low power modes 
  *         the software must first clear the RSF flag. 
  *         The software must then wait until it is set again before reading 
  *         the calendar, which means that the calendar registers have been 
  *         correctly copied into the RTC_TR and RTC_DR shadow registers.   
  * @param  None
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC registers are synchronised
  *          - ERROR: RTC registers are not synchronised
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WaitForSynchro() -> ErrorStatus {
    let mut synchrocounter: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    let mut synchrostatus: uint32_t = 0i32 as uint32_t;
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
              as *mut RTC_TypeDef)).CR & 0x20i32 as uint32_t !=
           RESET as libc::c_int as libc::c_uint {
        /* Bypass shadow mode */
        status = SUCCESS
    } else {
        /* Disable the write protection for RTC registers */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).WPR as
                                        *mut uint32_t, 0xcai32 as uint32_t);
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).WPR as
                                        *mut uint32_t, 0x53i32 as uint32_t);
        /* Clear RSF flag */
        let ref mut fresh6 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).ISR;
        ::core::ptr::write_volatile(fresh6,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh6
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & 0xffffff5fu32) as
                                        uint32_t as uint32_t);
        loop 
             /* Wait the registers to be synchronised */
             {
            synchrostatus =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).ISR & 0x20i32 as uint32_t;
            ::core::ptr::write_volatile(&mut synchrocounter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&synchrocounter
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if !(synchrocounter != 0x8000i32 as uint32_t &&
                     synchrostatus == 0i32 as libc::c_uint) {
                break ;
            }
        }
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).ISR & 0x20i32 as uint32_t !=
               RESET as libc::c_int as libc::c_uint {
            status = SUCCESS
        } else { status = ERROR }
        /* Enable the write protection for RTC registers */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).WPR as
                                        *mut uint32_t, 0xffi32 as uint32_t)
    }
    return status;
}
/* *
  * @brief  Enables or disables the RTC reference clock detection.
  * @param  NewState: new state of the RTC reference clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC reference clock detection is enabled
  *          - ERROR: RTC reference clock detection is disabled  
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_RefClockCmd(mut NewState: FunctionalState)
 -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Set Initialization mode */
    if RTC_EnterInitMode() as libc::c_uint ==
           ERROR as libc::c_int as libc::c_uint {
        status = ERROR
    } else {
        if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint
           {
            /* Enable the RTC reference clock detection */
            let ref mut fresh7 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh7,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh7
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x10i32 as uint32_t) as uint32_t
                                            as uint32_t)
        } else {
            /* Disable the RTC reference clock detection */
            let ref mut fresh8 =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).CR;
            ::core::ptr::write_volatile(fresh8,
                                        (::core::ptr::read_volatile::<uint32_t>(fresh8
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        /* Exit Initialization mode */
        RTC_ExitInitMode();
        status = SUCCESS
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @brief  Enables or Disables the Bypass Shadow feature.
  * @note   When the Bypass Shadow is enabled the calendar value are taken 
  *         directly from the Calendar counter.
  * @param  NewState: new state of the Bypass Shadow feature.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn RTC_BypassShadowCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the BYPSHAD bit */
        let ref mut fresh9 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh9,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh9
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20i32 as uint32_t as uint8_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    } else {
        /* Reset the BYPSHAD bit */
        let ref mut fresh10 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh10,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh10
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x20i32 as uint32_t) as uint8_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup RTC_Group2 Time and Date configuration functions
 *  @brief   Time and Date configuration functions 
 *
@verbatim   
 ===============================================================================
               ##### Time and Date configuration functions #####
 ===============================================================================  
    [..] This section provide functions allowing to program and read the RTC Calendar
         (Time and Date).

@endverbatim
  * @{
  */
/* *
  * @brief  Set the RTC current time.
  * @param  RTC_Format: specifies the format of the entered parameters.
  *   This parameter can be  one of the following values:
  *     @arg RTC_Format_BIN:  Binary data format 
  *     @arg RTC_Format_BCD:  BCD data format
  * @param  RTC_TimeStruct: pointer to a RTC_TimeTypeDef structure that contains 
  *                        the time configuration information for the RTC.     
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Time register is configured
  *          - ERROR: RTC Time register is not configured
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetTime(mut RTC_Format: uint32_t,
                                     mut RTC_TimeStruct: *mut RTC_TimeTypeDef)
 -> ErrorStatus {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    if RTC_Format == 0i32 as uint32_t {
        if !((*((0x40000000i32 as
                     uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                    *mut RTC_TypeDef)).CR & 0x40i32 as uint32_t !=
                 RESET as libc::c_int as uint32_t) {
            (*RTC_TimeStruct).RTC_H12 = 0i32 as uint8_t
        }
    } else if (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                     *mut RTC_TypeDef)).CR & 0x40i32 as uint32_t !=
                  RESET as libc::c_int as uint32_t {
        tmpreg = RTC_Bcd2ToByte((*RTC_TimeStruct).RTC_Hours) as uint32_t
    } else { (*RTC_TimeStruct).RTC_H12 = 0i32 as uint8_t }
    /* Check the input parameters format */
    if RTC_Format != 0i32 as uint32_t {
        tmpreg =
            ((*RTC_TimeStruct).RTC_Hours as uint32_t) << 16i32 |
                ((*RTC_TimeStruct).RTC_Minutes as uint32_t) << 8i32 |
                (*RTC_TimeStruct).RTC_Seconds as uint32_t |
                ((*RTC_TimeStruct).RTC_H12 as uint32_t) << 16i32
    } else {
        tmpreg =
            (RTC_ByteToBcd2((*RTC_TimeStruct).RTC_Hours) as uint32_t) << 16i32
                |
                (RTC_ByteToBcd2((*RTC_TimeStruct).RTC_Minutes) as uint32_t) <<
                    8i32 |
                RTC_ByteToBcd2((*RTC_TimeStruct).RTC_Seconds) as uint32_t |
                ((*RTC_TimeStruct).RTC_H12 as uint32_t) << 16i32
    }
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Set Initialization mode */
    if RTC_EnterInitMode() as libc::c_uint ==
           ERROR as libc::c_int as libc::c_uint {
        status = ERROR
    } else {
        /* Set the RTC_TR register */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).TR as
                                        *mut uint32_t,
                                    tmpreg & 0x7f7f7fi32 as uint32_t);
        /* Exit Initialization mode */
        RTC_ExitInitMode();
        /* If  RTC_CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).CR & 0x20i32 as uint32_t ==
               RESET as libc::c_int as libc::c_uint {
            if RTC_WaitForSynchro() as libc::c_uint ==
                   ERROR as libc::c_int as libc::c_uint {
                status = ERROR
            } else { status = SUCCESS }
        } else { status = SUCCESS }
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @brief  Fills each RTC_TimeStruct member with its default value
  *         (Time = 00h:00min:00sec).
  * @param  RTC_TimeStruct: pointer to a RTC_TimeTypeDef structure which will be 
  *         initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TimeStructInit(mut RTC_TimeStruct:
                                                *mut RTC_TimeTypeDef) {
    /* Time = 00h:00min:00sec */
    (*RTC_TimeStruct).RTC_H12 = 0i32 as uint8_t;
    (*RTC_TimeStruct).RTC_Hours = 0i32 as uint8_t;
    (*RTC_TimeStruct).RTC_Minutes = 0i32 as uint8_t;
    (*RTC_TimeStruct).RTC_Seconds = 0i32 as uint8_t;
}
/* *
  * @brief  Get the RTC current Time.
  * @param  RTC_Format: specifies the format of the returned parameters.
  *   This parameter can be  one of the following values:
  *     @arg RTC_Format_BIN:  Binary data format 
  *     @arg RTC_Format_BCD:  BCD data format
  * @param RTC_TimeStruct: pointer to a RTC_TimeTypeDef structure that will 
  *                        contain the returned current time configuration.     
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetTime(mut RTC_Format: uint32_t,
                                     mut RTC_TimeStruct:
                                         *mut RTC_TimeTypeDef) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the RTC_TR register */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TR & 0x7f7f7fi32 as uint32_t;
    /* Fill the structure fields with the read parameters */
    (*RTC_TimeStruct).RTC_Hours =
        ((tmpreg & (0x300000i32 as uint32_t | 0xf0000i32 as uint32_t)) >>
             16i32) as uint8_t;
    (*RTC_TimeStruct).RTC_Minutes =
        ((tmpreg & (0x7000i32 as uint32_t | 0xf00i32 as uint32_t)) >> 8i32) as
            uint8_t;
    (*RTC_TimeStruct).RTC_Seconds =
        (tmpreg & (0x70i32 as uint32_t | 0xfi32 as uint32_t)) as uint8_t;
    (*RTC_TimeStruct).RTC_H12 =
        ((tmpreg & 0x400000i32 as uint32_t) >> 16i32) as uint8_t;
    /* Check the input parameters format */
    if RTC_Format == 0i32 as uint32_t {
        /* Convert the structure parameters to Binary format */
        (*RTC_TimeStruct).RTC_Hours =
            RTC_Bcd2ToByte((*RTC_TimeStruct).RTC_Hours);
        (*RTC_TimeStruct).RTC_Minutes =
            RTC_Bcd2ToByte((*RTC_TimeStruct).RTC_Minutes);
        (*RTC_TimeStruct).RTC_Seconds =
            RTC_Bcd2ToByte((*RTC_TimeStruct).RTC_Seconds)
    };
}
/* *
  * @brief  Gets the RTC current Calendar Subseconds value.
  * @note   This function freeze the Time and Date registers after reading the 
  *         SSR register.
  * @param  None
  * @retval RTC current Calendar Subseconds value.
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetSubSecond() -> uint32_t {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Get subseconds values from the correspondent registers*/
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).SSR;
    /* Read DR register to unfroze calendar registers */
    return tmpreg;
}
/* *
  * @brief  Set the RTC current date.
  * @param  RTC_Format: specifies the format of the entered parameters.
  *   This parameter can be  one of the following values:
  *     @arg RTC_Format_BIN:  Binary data format 
  *     @arg RTC_Format_BCD:  BCD data format
  * @param  RTC_DateStruct: pointer to a RTC_DateTypeDef structure that contains 
  *                         the date configuration information for the RTC.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Date register is configured
  *          - ERROR: RTC Date register is not configured
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetDate(mut RTC_Format: uint32_t,
                                     mut RTC_DateStruct: *mut RTC_DateTypeDef)
 -> ErrorStatus {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    if RTC_Format == 0i32 as uint32_t &&
           (*RTC_DateStruct).RTC_Month as libc::c_int & 0x10i32 == 0x10i32 {
        (*RTC_DateStruct).RTC_Month =
            ((*RTC_DateStruct).RTC_Month as libc::c_uint &
                 !0x10i32 as uint32_t).wrapping_add(0xai32 as libc::c_uint) as
                uint8_t
    }
    if !(RTC_Format == 0i32 as uint32_t) {
        tmpreg = RTC_Bcd2ToByte((*RTC_DateStruct).RTC_Month) as uint32_t;
        tmpreg = RTC_Bcd2ToByte((*RTC_DateStruct).RTC_Date) as uint32_t
    }
    /* Check the input parameters format */
    if RTC_Format != 0i32 as uint32_t {
        tmpreg =
            ((*RTC_DateStruct).RTC_Year as uint32_t) << 16i32 |
                ((*RTC_DateStruct).RTC_Month as uint32_t) << 8i32 |
                (*RTC_DateStruct).RTC_Date as uint32_t |
                ((*RTC_DateStruct).RTC_WeekDay as uint32_t) << 13i32
    } else {
        tmpreg =
            (RTC_ByteToBcd2((*RTC_DateStruct).RTC_Year) as uint32_t) << 16i32
                |
                (RTC_ByteToBcd2((*RTC_DateStruct).RTC_Month) as uint32_t) <<
                    8i32 |
                RTC_ByteToBcd2((*RTC_DateStruct).RTC_Date) as uint32_t |
                ((*RTC_DateStruct).RTC_WeekDay as uint32_t) << 13i32
    }
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Set Initialization mode */
    if RTC_EnterInitMode() as libc::c_uint ==
           ERROR as libc::c_int as libc::c_uint {
        status = ERROR
    } else {
        /* Set the RTC_DR register */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).DR as
                                        *mut uint32_t,
                                    tmpreg & 0xffff3fi32 as uint32_t);
        /* Exit Initialization mode */
        RTC_ExitInitMode();
        /* If  RTC_CR_BYPSHAD bit = 0, wait for synchro else this check is not needed */
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).CR & 0x20i32 as uint32_t ==
               RESET as libc::c_int as libc::c_uint {
            if RTC_WaitForSynchro() as libc::c_uint ==
                   ERROR as libc::c_int as libc::c_uint {
                status = ERROR
            } else { status = SUCCESS }
        } else { status = SUCCESS }
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @brief  Fills each RTC_DateStruct member with its default value
  *         (Monday, January 01 xx00).
  * @param  RTC_DateStruct: pointer to a RTC_DateTypeDef structure which will be 
  *         initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_DateStructInit(mut RTC_DateStruct:
                                                *mut RTC_DateTypeDef) {
    /* Monday, January 01 xx00 */
    (*RTC_DateStruct).RTC_WeekDay = 0x1i32 as uint8_t;
    (*RTC_DateStruct).RTC_Date = 1i32 as uint8_t;
    (*RTC_DateStruct).RTC_Month = 0x1i32 as uint8_t;
    (*RTC_DateStruct).RTC_Year = 0i32 as uint8_t;
}
/* *
  * @brief  Get the RTC current date.
  * @param  RTC_Format: specifies the format of the returned parameters.
  *   This parameter can be one of the following values:
  *     @arg RTC_Format_BIN: Binary data format 
  *     @arg RTC_Format_BCD: BCD data format
  * @param RTC_DateStruct: pointer to a RTC_DateTypeDef structure that will 
  *                        contain the returned current date configuration.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetDate(mut RTC_Format: uint32_t,
                                     mut RTC_DateStruct:
                                         *mut RTC_DateTypeDef) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the RTC_TR register */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).DR & 0xffff3fi32 as uint32_t;
    /* Fill the structure fields with the read parameters */
    (*RTC_DateStruct).RTC_Year =
        ((tmpreg & (0xf00000i32 as uint32_t | 0xf0000i32 as uint32_t)) >>
             16i32) as uint8_t;
    (*RTC_DateStruct).RTC_Month =
        ((tmpreg & (0x1000i32 as uint32_t | 0xf00i32 as uint32_t)) >> 8i32) as
            uint8_t;
    (*RTC_DateStruct).RTC_Date =
        (tmpreg & (0x30i32 as uint32_t | 0xfi32 as uint32_t)) as uint8_t;
    (*RTC_DateStruct).RTC_WeekDay =
        ((tmpreg & 0xe000i32 as uint32_t) >> 13i32) as uint8_t;
    /* Check the input parameters format */
    if RTC_Format == 0i32 as uint32_t {
        /* Convert the structure parameters to Binary format */
        (*RTC_DateStruct).RTC_Year =
            RTC_Bcd2ToByte((*RTC_DateStruct).RTC_Year);
        (*RTC_DateStruct).RTC_Month =
            RTC_Bcd2ToByte((*RTC_DateStruct).RTC_Month);
        (*RTC_DateStruct).RTC_Date =
            RTC_Bcd2ToByte((*RTC_DateStruct).RTC_Date);
        (*RTC_DateStruct).RTC_WeekDay = (*RTC_DateStruct).RTC_WeekDay
    };
}
/* *
  * @}
  */
/* * @defgroup RTC_Group3 Alarms configuration functions
 *  @brief   Alarms (Alarm A and Alarm B) configuration functions 
 *
@verbatim   
 ===============================================================================
        ##### Alarms (Alarm A and Alarm B) configuration functions #####
 ===============================================================================  
    [..] This section provides functions allowing to program and read the RTC Alarms.

@endverbatim
  * @{
  */
/* *
  * @brief  Set the specified RTC Alarm.
  * @note   The Alarm register can only be written when the corresponding Alarm
  *         is disabled (Use the RTC_AlarmCmd(DISABLE)).    
  * @param  RTC_Format: specifies the format of the returned parameters.
  *   This parameter can be one of the following values:
  *     @arg RTC_Format_BIN: Binary data format 
  *     @arg RTC_Format_BCD: BCD data format
  * @param  RTC_Alarm: specifies the alarm to be configured.
  *   This parameter can be one of the following values:
  *     @arg RTC_Alarm_A: to select Alarm A
  *     @arg RTC_Alarm_B: to select Alarm B  
  * @param  RTC_AlarmStruct: pointer to a RTC_AlarmTypeDef structure that 
  *                          contains the alarm configuration parameters.     
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetAlarm(mut RTC_Format: uint32_t,
                                      mut RTC_Alarm: uint32_t,
                                      mut RTC_AlarmStruct:
                                          *mut RTC_AlarmTypeDef) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    if RTC_Format == 0i32 as uint32_t {
        if !((*((0x40000000i32 as
                     uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                    *mut RTC_TypeDef)).CR & 0x40i32 as uint32_t !=
                 RESET as libc::c_int as uint32_t) {
            (*RTC_AlarmStruct).RTC_AlarmTime.RTC_H12 = 0i32 as uint8_t
        }
        ((*RTC_AlarmStruct).RTC_AlarmDateWeekDaySel) == 0i32 as uint32_t;
    } else {
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).CR & 0x40i32 as uint32_t !=
               RESET as libc::c_int as uint32_t {
            tmpreg =
                RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours) as
                    uint32_t
        } else { (*RTC_AlarmStruct).RTC_AlarmTime.RTC_H12 = 0i32 as uint8_t }
        if (*RTC_AlarmStruct).RTC_AlarmDateWeekDaySel == 0i32 as uint32_t {
            tmpreg =
                RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmDateWeekDay) as
                    uint32_t
        } else {
            tmpreg =
                RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmDateWeekDay) as
                    uint32_t
        }
    }
    /* Check the input parameters format */
    if RTC_Format != 0i32 as uint32_t {
        tmpreg =
            ((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours as uint32_t) << 16i32
                |
                ((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Minutes as uint32_t) <<
                    8i32 |
                (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Seconds as uint32_t |
                ((*RTC_AlarmStruct).RTC_AlarmTime.RTC_H12 as uint32_t) <<
                    16i32 |
                ((*RTC_AlarmStruct).RTC_AlarmDateWeekDay as uint32_t) << 24i32
                | (*RTC_AlarmStruct).RTC_AlarmDateWeekDaySel |
                (*RTC_AlarmStruct).RTC_AlarmMask
    } else {
        tmpreg =
            (RTC_ByteToBcd2((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours) as
                 uint32_t) << 16i32 |
                (RTC_ByteToBcd2((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Minutes)
                     as uint32_t) << 8i32 |
                RTC_ByteToBcd2((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Seconds)
                    as uint32_t |
                ((*RTC_AlarmStruct).RTC_AlarmTime.RTC_H12 as uint32_t) <<
                    16i32 |
                (RTC_ByteToBcd2((*RTC_AlarmStruct).RTC_AlarmDateWeekDay) as
                     uint32_t) << 24i32 |
                (*RTC_AlarmStruct).RTC_AlarmDateWeekDaySel |
                (*RTC_AlarmStruct).RTC_AlarmMask
    }
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Configure the Alarm register */
    if RTC_Alarm == 0x100i32 as uint32_t {
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).ALRMAR
                                        as *mut uint32_t, tmpreg)
    } else {
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).ALRMBR
                                        as *mut uint32_t, tmpreg)
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Fills each RTC_AlarmStruct member with its default value
  *         (Time = 00h:00mn:00sec / Date = 1st day of the month/Mask =
  *         all fields are masked).
  * @param  RTC_AlarmStruct: pointer to a @ref RTC_AlarmTypeDef structure which
  *         will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_AlarmStructInit(mut RTC_AlarmStruct:
                                                 *mut RTC_AlarmTypeDef) {
    /* Alarm Time Settings : Time = 00h:00mn:00sec */
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_H12 = 0i32 as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours = 0i32 as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Minutes = 0i32 as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Seconds = 0i32 as uint8_t;
    /* Alarm Date Settings : Date = 1st day of the month */
    (*RTC_AlarmStruct).RTC_AlarmDateWeekDaySel = 0i32 as uint32_t;
    (*RTC_AlarmStruct).RTC_AlarmDateWeekDay = 1i32 as uint8_t;
    /* Alarm Masks Settings : Mask =  all fields are not masked */
    (*RTC_AlarmStruct).RTC_AlarmMask = 0i32 as uint32_t;
}
/* *
  * @brief  Get the RTC Alarm value and masks.
  * @param  RTC_Format: specifies the format of the output parameters.
  *   This parameter can be one of the following values:
  *     @arg RTC_Format_BIN: Binary data format 
  *     @arg RTC_Format_BCD: BCD data format
  * @param  RTC_Alarm: specifies the alarm to be read.
  *   This parameter can be one of the following values:
  *     @arg RTC_Alarm_A: to select Alarm A
  *     @arg RTC_Alarm_B: to select Alarm B  
  * @param  RTC_AlarmStruct: pointer to a RTC_AlarmTypeDef structure that will 
  *                          contains the output alarm configuration values.     
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetAlarm(mut RTC_Format: uint32_t,
                                      mut RTC_Alarm: uint32_t,
                                      mut RTC_AlarmStruct:
                                          *mut RTC_AlarmTypeDef) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the RTC_ALRMxR register */
    if RTC_Alarm == 0x100i32 as uint32_t {
        tmpreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).ALRMAR
    } else {
        tmpreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).ALRMBR
    }
    /* Fill the structure with the read parameters */
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours =
        ((tmpreg & (0x300000i32 as uint32_t | 0xf0000i32 as uint32_t)) >>
             16i32) as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Minutes =
        ((tmpreg & (0x7000i32 as uint32_t | 0xf00i32 as uint32_t)) >> 8i32) as
            uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Seconds =
        (tmpreg & (0x70i32 as uint32_t | 0xfi32 as uint32_t)) as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmTime.RTC_H12 =
        ((tmpreg & 0x400000i32 as uint32_t) >> 16i32) as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmDateWeekDay =
        ((tmpreg & (0x30000000i32 as uint32_t | 0xf000000i32 as uint32_t)) >>
             24i32) as uint8_t;
    (*RTC_AlarmStruct).RTC_AlarmDateWeekDaySel =
        tmpreg & 0x40000000i32 as uint32_t;
    (*RTC_AlarmStruct).RTC_AlarmMask = tmpreg & 0x80808080u32;
    if RTC_Format == 0i32 as uint32_t {
        (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours =
            RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Hours);
        (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Minutes =
            RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Minutes);
        (*RTC_AlarmStruct).RTC_AlarmTime.RTC_Seconds =
            RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmTime.RTC_Seconds);
        (*RTC_AlarmStruct).RTC_AlarmDateWeekDay =
            RTC_Bcd2ToByte((*RTC_AlarmStruct).RTC_AlarmDateWeekDay)
    };
}
/* *
  * @brief  Enables or disables the specified RTC Alarm.
  * @param  RTC_Alarm: specifies the alarm to be configured.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_Alarm_A: to select Alarm A
  *     @arg RTC_Alarm_B: to select Alarm B  
  * @param  NewState: new state of the specified alarm.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Alarm is enabled/disabled
  *          - ERROR: RTC Alarm is not enabled/disabled  
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_AlarmCmd(mut RTC_Alarm: uint32_t,
                                      mut NewState: FunctionalState)
 -> ErrorStatus {
    let mut alarmcounter: uint32_t = 0i32 as uint32_t;
    let mut alarmstatus: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Configure the Alarm state */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh11 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh11,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh11
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RTC_Alarm) as
                                        uint32_t as uint32_t);
        status = SUCCESS
    } else {
        /* Disable the Alarm in RTC_CR register */
        let ref mut fresh12 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh12,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh12
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RTC_Alarm) as
                                        uint32_t as uint32_t);
        loop 
             /* Wait till RTC ALRxWF flag is set and if Time out is reached exit */
             {
            alarmstatus =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).ISR & RTC_Alarm >> 8i32;
            ::core::ptr::write_volatile(&mut alarmcounter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&alarmcounter
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if !(alarmcounter != 0x2000i32 as uint32_t &&
                     alarmstatus == 0i32 as libc::c_uint) {
                break ;
            }
        }
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).ISR & RTC_Alarm >> 8i32 ==
               RESET as libc::c_int as libc::c_uint {
            status = ERROR
        } else { status = SUCCESS }
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @brief  Configures the RTC AlarmA/B Subseconds value and mask.
  * @note   This function is performed only when the Alarm is disabled. 
  * @param  RTC_Alarm: specifies the alarm to be configured.
  *   This parameter can be one of the following values:
  *     @arg RTC_Alarm_A: to select Alarm A
  *     @arg RTC_Alarm_B: to select Alarm B
  * @param  RTC_AlarmSubSecondValue: specifies the Subseconds value.
  *   This parameter can be a value from 0 to 0x00007FFF.
  * @param  RTC_AlarmSubSecondMask:  specifies the Subseconds Mask.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_AlarmSubSecondMask_All    : All Alarm SS fields are masked.
  *                                          There is no comparison on sub seconds for Alarm.
  *     @arg RTC_AlarmSubSecondMask_SS14_1 : SS[14:1] are don't care in Alarm comparison.
  *                                          Only SS[0] is compared
  *     @arg RTC_AlarmSubSecondMask_SS14_2 : SS[14:2] are don't care in Alarm comparison.
  *                                          Only SS[1:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_3 : SS[14:3] are don't care in Alarm comparison.
  *                                          Only SS[2:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_4 : SS[14:4] are don't care in Alarm comparison.
  *                                          Only SS[3:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_5 : SS[14:5] are don't care in Alarm comparison.
  *                                          Only SS[4:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_6 : SS[14:6] are don't care in Alarm comparison.
  *                                          Only SS[5:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_7 : SS[14:7] are don't care in Alarm comparison.
  *                                          Only SS[6:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_8 : SS[14:8] are don't care in Alarm comparison.
  *                                          Only SS[7:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_9 : SS[14:9] are don't care in Alarm comparison.
  *                                          Only SS[8:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_10: SS[14:10] are don't care in Alarm comparison.
  *                                          Only SS[9:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_11: SS[14:11] are don't care in Alarm comparison.
  *                                          Only SS[10:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_12: SS[14:12] are don't care in Alarm comparison.
  *                                          Only SS[11:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14_13: SS[14:13] are don't care in Alarm comparison.
  *                                          Only SS[12:0] are compared
  *     @arg RTC_AlarmSubSecondMask_SS14   : SS[14] is don't care in Alarm comparison.
  *                                          Only SS[13:0] are compared
  *     @arg RTC_AlarmSubSecondMask_None   : SS[14:0] are compared and must match
  *                                          to activate alarm
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_AlarmSubSecondConfig(mut RTC_Alarm: uint32_t,
                                                  mut RTC_AlarmSubSecondValue:
                                                      uint32_t,
                                                  mut RTC_AlarmSubSecondMask:
                                                      uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Configure the Alarm A or Alarm B SubSecond registers */
    tmpreg = RTC_AlarmSubSecondValue | RTC_AlarmSubSecondMask;
    if RTC_Alarm == 0x100i32 as uint32_t {
        /* Configure the AlarmA SubSecond register */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).ALRMASSR
                                        as *mut uint32_t, tmpreg)
    } else {
        /* Configure the Alarm B SubSecond register */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).ALRMBSSR
                                        as *mut uint32_t, tmpreg)
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Gets the RTC Alarm Subseconds value.
  * @param  RTC_Alarm: specifies the alarm to be read.
  *   This parameter can be one of the following values:
  *     @arg RTC_Alarm_A: to select Alarm A
  *     @arg RTC_Alarm_B: to select Alarm B
  * @param  None
  * @retval RTC Alarm Subseconds value.
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetAlarmSubSecond(mut RTC_Alarm: uint32_t)
 -> uint32_t {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Get the RTC_ALRMxR register */
    if RTC_Alarm == 0x100i32 as uint32_t {
        tmpreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).ALRMASSR & 0x7fffi32 as uint32_t
    } else {
        tmpreg =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).ALRMBSSR & 0x7fffi32 as uint32_t
    }
    return tmpreg;
}
/* *
  * @}
  */
/* * @defgroup RTC_Group4 WakeUp Timer configuration functions
 *  @brief   WakeUp Timer configuration functions 
 *
@verbatim   
 ===============================================================================
                ##### WakeUp Timer configuration functions #####
 ===============================================================================  
    [..] This section provide functions allowing to program and read the RTC WakeUp.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the RTC Wakeup clock source.
  * @note   The WakeUp Clock source can only be changed when the RTC WakeUp
  *         is disabled (Use the RTC_WakeUpCmd(DISABLE)).
  * @param  RTC_WakeUpClock: Wakeup Clock source.
  *   This parameter can be one of the following values:
  *     @arg RTC_WakeUpClock_RTCCLK_Div16: RTC Wakeup Counter Clock = RTCCLK/16
  *     @arg RTC_WakeUpClock_RTCCLK_Div8: RTC Wakeup Counter Clock = RTCCLK/8
  *     @arg RTC_WakeUpClock_RTCCLK_Div4: RTC Wakeup Counter Clock = RTCCLK/4
  *     @arg RTC_WakeUpClock_RTCCLK_Div2: RTC Wakeup Counter Clock = RTCCLK/2
  *     @arg RTC_WakeUpClock_CK_SPRE_16bits: RTC Wakeup Counter Clock = CK_SPRE
  *     @arg RTC_WakeUpClock_CK_SPRE_17bits: RTC Wakeup Counter Clock = CK_SPRE
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WakeUpClockConfig(mut RTC_WakeUpClock:
                                                   uint32_t) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Clear the Wakeup Timer clock source bits in CR register */
    let ref mut fresh13 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh13,
                                (::core::ptr::read_volatile::<uint32_t>(fresh13
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x7i32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Configure the clock source */
    let ref mut fresh14 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh14,
                                (::core::ptr::read_volatile::<uint32_t>(fresh14
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RTC_WakeUpClock) as
                                    uint32_t as uint32_t);
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Configures the RTC Wakeup counter.
  * @note   The RTC WakeUp counter can only be written when the RTC WakeUp
  *         is disabled (Use the RTC_WakeUpCmd(DISABLE)).
  * @param  RTC_WakeUpCounter: specifies the WakeUp counter.
  *   This parameter can be a value from 0x0000 to 0xFFFF. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetWakeUpCounter(mut RTC_WakeUpCounter:
                                                  uint32_t) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Configure the Wakeup Timer counter */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WUTR as
                                    *mut uint32_t, RTC_WakeUpCounter);
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Returns the RTC WakeUp timer counter value.
  * @param  None
  * @retval The RTC WakeUp Counter value.
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetWakeUpCounter() -> uint32_t {
    /* Get the counter value */
    return (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).WUTR & 0xffffi32 as uint32_t;
}
/* *
  * @brief  Enables or Disables the RTC WakeUp timer.
  * @param  NewState: new state of the WakeUp timer.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WakeUpCmd(mut NewState: FunctionalState)
 -> ErrorStatus {
    let mut wutcounter: uint32_t = 0i32 as uint32_t;
    let mut wutwfstatus: uint32_t = 0i32 as uint32_t;
    let mut status: ErrorStatus = ERROR;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Wakeup Timer */
        let ref mut fresh15 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh15,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh15
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x400i32 as uint32_t) as uint32_t as
                                        uint32_t);
        status = SUCCESS
    } else {
        /* Disable the Wakeup Timer */
        let ref mut fresh16 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh16,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh16
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x400i32 as uint32_t)) as uint32_t
                                        as uint32_t);
        loop 
             /* Wait till RTC WUTWF flag is set and if Time out is reached exit */
             {
            wutwfstatus =
                (*((0x40000000i32 as
                        uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                       *mut RTC_TypeDef)).ISR & 0x4i32 as uint32_t;
            ::core::ptr::write_volatile(&mut wutcounter as *mut uint32_t,
                                        ::core::ptr::read_volatile::<uint32_t>(&wutcounter
                                                                                   as
                                                                                   *const uint32_t).wrapping_add(1));
            if !(wutcounter != 0x2000i32 as uint32_t &&
                     wutwfstatus == 0i32 as libc::c_uint) {
                break ;
            }
        }
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).ISR & 0x4i32 as uint32_t ==
               RESET as libc::c_int as libc::c_uint {
            status = ERROR
        } else { status = SUCCESS }
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @}
  */
/* * @defgroup RTC_Group5 Daylight Saving configuration functions
 *  @brief   Daylight Saving configuration functions 
 *
@verbatim   
 ===============================================================================
                ##### Daylight Saving configuration functions #####
 ===============================================================================  
    [..] This section provide functions allowing to configure the RTC DayLight Saving.

@endverbatim
  * @{
  */
/* *
  * @brief  Adds or substract one hour from the current time.
  * @param  RTC_DayLightSaveOperation: the value of hour adjustment. 
  *   This parameter can be one of the following values:
  *     @arg RTC_DayLightSaving_SUB1H: Substract one hour (winter time)
  *     @arg RTC_DayLightSaving_ADD1H: Add one hour (summer time)
  * @param  RTC_StoreOperation: Specifies the value to be written in the BCK bit 
  *                            in CR register to store the operation.
  *   This parameter can be one of the following values:
  *     @arg RTC_StoreOperation_Reset: BCK Bit Reset
  *     @arg RTC_StoreOperation_Set: BCK Bit Set
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_DayLightSavingConfig(mut RTC_DayLightSaving:
                                                      uint32_t,
                                                  mut RTC_StoreOperation:
                                                      uint32_t) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Clear the bits to be configured */
    let ref mut fresh17 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh17,
                                (::core::ptr::read_volatile::<uint32_t>(fresh17
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x40000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Configure the RTC_CR register */
    let ref mut fresh18 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh18,
                                (::core::ptr::read_volatile::<uint32_t>(fresh18
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (RTC_DayLightSaving |
                                          RTC_StoreOperation)) as uint32_t as
                                    uint32_t);
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Returns the RTC Day Light Saving stored operation.
  * @param  None
  * @retval RTC Day Light Saving stored operation.
  *          - RTC_StoreOperation_Reset
  *          - RTC_StoreOperation_Set
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetStoreOperation() -> uint32_t {
    return (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).CR & 0x40000i32 as uint32_t;
}
/* *
  * @}
  */
/* * @defgroup RTC_Group6 Output pin Configuration function
 *  @brief   Output pin Configuration function 
 *
@verbatim   
 ===============================================================================
                   ##### Output pin Configuration function #####
 ===============================================================================
    [..] This section provide functions allowing to configure the RTC Output source.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the RTC output source (AFO_ALARM).
  * @param  RTC_Output: Specifies which signal will be routed to the RTC output. 
  *   This parameter can be one of the following values:
  *     @arg RTC_Output_Disable: No output selected
  *     @arg RTC_Output_AlarmA: signal of AlarmA mapped to output
  *     @arg RTC_Output_AlarmB: signal of AlarmB mapped to output
  *     @arg RTC_Output_WakeUp: signal of WakeUp mapped to output
  * @param  RTC_OutputPolarity: Specifies the polarity of the output signal. 
  *   This parameter can be one of the following:
  *     @arg RTC_OutputPolarity_High: The output pin is high when the 
  *                                 ALRAF/ALRBF/WUTF is high (depending on OSEL)
  *     @arg RTC_OutputPolarity_Low: The output pin is low when the 
  *                                 ALRAF/ALRBF/WUTF is high (depending on OSEL)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_OutputConfig(mut RTC_Output: uint32_t,
                                          mut RTC_OutputPolarity: uint32_t) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Clear the bits to be configured */
    let ref mut fresh19 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh19,
                                (::core::ptr::read_volatile::<uint32_t>(fresh19
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x600000i32 as uint32_t |
                                           0x100000i32 as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Configure the output selection and polarity */
    let ref mut fresh20 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh20,
                                (::core::ptr::read_volatile::<uint32_t>(fresh20
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (RTC_Output | RTC_OutputPolarity)) as
                                    uint32_t as uint32_t);
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup RTC_Group7 Digital Calibration configuration functions
 *  @brief   Digital Calibration configuration functions 
 *
@verbatim   
 ===============================================================================
            ##### Digital Calibration configuration functions #####
 ===============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the RTC clock to be output through the relative 
  *         pin.
  * @param  NewState: new state of the digital calibration Output.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_CalibOutputCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the RTC clock output */
        let ref mut fresh21 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh21,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh21
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x800000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the RTC clock output */
        let ref mut fresh22 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh22,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh22
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x800000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Configures the Calibration Pinout (RTC_CALIB) Selection (1Hz or 512Hz).
  * @param  RTC_CalibOutput : Select the Calibration output Selection .
  *   This parameter can be one of the following values:
  *     @arg RTC_CalibOutput_512Hz: A signal has a regular waveform at 512Hz. 
  *     @arg RTC_CalibOutput_1Hz  : A signal has a regular waveform at 1Hz.
  * @retval None
*/
#[no_mangle]
pub unsafe extern "C" fn RTC_CalibOutputConfig(mut RTC_CalibOutput:
                                                   uint32_t) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /*clear flags before config*/
    let ref mut fresh23 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh23,
                                (::core::ptr::read_volatile::<uint32_t>(fresh23
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x80000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Configure the RTC_CR register */
    let ref mut fresh24 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR;
    ::core::ptr::write_volatile(fresh24,
                                (::core::ptr::read_volatile::<uint32_t>(fresh24
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RTC_CalibOutput) as
                                    uint32_t as uint32_t);
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Configures the Smooth Calibration Settings.
  * @param  RTC_SmoothCalibPeriod : Select the Smooth Calibration Period.
  *   This parameter can be can be one of the following values:
  *     @arg RTC_SmoothCalibPeriod_32sec : The smooth calibration periode is 32s.
  *     @arg RTC_SmoothCalibPeriod_16sec : The smooth calibration periode is 16s.
  *     @arg RTC_SmoothCalibPeriod_8sec  : The smooth calibartion periode is 8s.
  * @param  RTC_SmoothCalibPlusPulses : Select to Set or reset the CALP bit.
  *   This parameter can be one of the following values:
  *     @arg RTC_SmoothCalibPlusPulses_Set  : Add one RTCCLK puls every 2**11 pulses.
  *     @arg RTC_SmoothCalibPlusPulses_Reset: No RTCCLK pulses are added.
  * @param  RTC_SmouthCalibMinusPulsesValue: Select the value of CALM[8:0] bits.
  *   This parameter can be one any value from 0 to 0x000001FF.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Calib registers are configured
  *          - ERROR: RTC Calib registers are not configured
*/
#[no_mangle]
pub unsafe extern "C" fn RTC_SmoothCalibConfig(mut RTC_SmoothCalibPeriod:
                                                   uint32_t,
                                               mut RTC_SmoothCalibPlusPulses:
                                                   uint32_t,
                                               mut RTC_SmouthCalibMinusPulsesValue:
                                                   uint32_t) -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    let mut recalpfcount: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* check if a calibration is pending*/
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
              as *mut RTC_TypeDef)).ISR & 0x10000i32 as uint32_t !=
           RESET as libc::c_int as libc::c_uint {
        /* wait until the Calibration is completed*/
        while (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                     *mut RTC_TypeDef)).ISR & 0x10000i32 as uint32_t !=
                  RESET as libc::c_int as libc::c_uint &&
                  recalpfcount != 0x1000i32 as uint32_t {
            recalpfcount = recalpfcount.wrapping_add(1)
        }
    }
    /* check if the calibration pending is completed or if there is no calibration operation at all*/
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
              as *mut RTC_TypeDef)).ISR & 0x10000i32 as uint32_t ==
           RESET as libc::c_int as libc::c_uint {
        /* Configure the Smooth calibration settings */
        ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                 uint32_t).wrapping_add(0x2800i32
                                                                            as
                                                                            libc::c_uint)
                                                as *mut RTC_TypeDef)).CALR as
                                        *mut uint32_t,
                                    RTC_SmoothCalibPeriod |
                                        RTC_SmoothCalibPlusPulses |
                                        RTC_SmouthCalibMinusPulsesValue);
        status = SUCCESS
    } else { status = ERROR }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @}
  */
/* * @defgroup RTC_Group8 TimeStamp configuration functions
 *  @brief   TimeStamp configuration functions 
 *
@verbatim   
 ===============================================================================
                ##### TimeStamp configuration functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or Disables the RTC TimeStamp functionality with the 
  *         specified time stamp pin stimulating edge.
  * @param  RTC_TimeStampEdge: Specifies the pin edge on which the TimeStamp is 
  *         activated.
  *   This parameter can be one of the following:
  *     @arg RTC_TimeStampEdge_Rising: the Time stamp event occurs on the rising 
  *                                    edge of the related pin.
  *     @arg RTC_TimeStampEdge_Falling: the Time stamp event occurs on the 
  *                                     falling edge of the related pin.
  * @param  NewState: new state of the TimeStamp.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TimeStampCmd(mut RTC_TimeStampEdge: uint32_t,
                                          mut NewState: FunctionalState) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the RTC_CR register and clear the bits to be configured */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR &
            !(0x8i32 as uint32_t | 0x800i32 as uint32_t);
    /* Get the new configuration */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        tmpreg |= RTC_TimeStampEdge | 0x800i32 as uint32_t
    } else { tmpreg |= RTC_TimeStampEdge }
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Configure the Time Stamp TSEDGE and Enable bits */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).CR as
                                    *mut uint32_t, tmpreg);
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Gets the RTC TimeStamp value and masks.
  * @param  RTC_Format: specifies the format of the output parameters.
  *   This parameter can be one of the following values:
  *     @arg RTC_Format_BIN: Binary data format 
  *     @arg RTC_Format_BCD: BCD data format
  * @param RTC_StampTimeStruct: pointer to a RTC_TimeTypeDef structure that will 
  *                             contains the TimeStamp time values. 
  * @param RTC_StampDateStruct: pointer to a RTC_DateTypeDef structure that will 
  *                             contains the TimeStamp date values.     
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetTimeStamp(mut RTC_Format: uint32_t,
                                          mut RTC_StampTimeStruct:
                                              *mut RTC_TimeTypeDef,
                                          mut RTC_StampDateStruct:
                                              *mut RTC_DateTypeDef) {
    let mut tmptime: uint32_t = 0i32 as uint32_t;
    let mut tmpdate: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the TimeStamp time and date registers values */
    tmptime =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TSTR & 0x7f7f7fi32 as uint32_t;
    tmpdate =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TSDR & 0xffff3fi32 as uint32_t;
    /* Fill the Time structure fields with the read parameters */
    (*RTC_StampTimeStruct).RTC_Hours =
        ((tmptime & (0x300000i32 as uint32_t | 0xf0000i32 as uint32_t)) >>
             16i32) as uint8_t;
    (*RTC_StampTimeStruct).RTC_Minutes =
        ((tmptime & (0x7000i32 as uint32_t | 0xf00i32 as uint32_t)) >> 8i32)
            as uint8_t;
    (*RTC_StampTimeStruct).RTC_Seconds =
        (tmptime & (0x70i32 as uint32_t | 0xfi32 as uint32_t)) as uint8_t;
    (*RTC_StampTimeStruct).RTC_H12 =
        ((tmptime & 0x400000i32 as uint32_t) >> 16i32) as uint8_t;
    /* Fill the Date structure fields with the read parameters */
    (*RTC_StampDateStruct).RTC_Year = 0i32 as uint8_t;
    (*RTC_StampDateStruct).RTC_Month =
        ((tmpdate & (0x1000i32 as uint32_t | 0xf00i32 as uint32_t)) >> 8i32)
            as uint8_t;
    (*RTC_StampDateStruct).RTC_Date =
        (tmpdate & (0x30i32 as uint32_t | 0xfi32 as uint32_t)) as uint8_t;
    (*RTC_StampDateStruct).RTC_WeekDay =
        ((tmpdate & 0xe000i32 as uint32_t) >> 13i32) as uint8_t;
    /* Check the input parameters format */
    if RTC_Format == 0i32 as uint32_t {
        /* Convert the Time structure parameters to Binary format */
        (*RTC_StampTimeStruct).RTC_Hours =
            RTC_Bcd2ToByte((*RTC_StampTimeStruct).RTC_Hours);
        (*RTC_StampTimeStruct).RTC_Minutes =
            RTC_Bcd2ToByte((*RTC_StampTimeStruct).RTC_Minutes);
        (*RTC_StampTimeStruct).RTC_Seconds =
            RTC_Bcd2ToByte((*RTC_StampTimeStruct).RTC_Seconds);
        /* Convert the Date structure parameters to Binary format */
        (*RTC_StampDateStruct).RTC_Month =
            RTC_Bcd2ToByte((*RTC_StampDateStruct).RTC_Month);
        (*RTC_StampDateStruct).RTC_Date =
            RTC_Bcd2ToByte((*RTC_StampDateStruct).RTC_Date);
        (*RTC_StampDateStruct).RTC_WeekDay =
            RTC_Bcd2ToByte((*RTC_StampDateStruct).RTC_WeekDay)
    };
}
/* *
  * @brief  Gets the RTC timestamp Subseconds value.
  * @param  None
  * @retval RTC current timestamp Subseconds value.
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetTimeStampSubSecond() -> uint32_t {
    /* Get timestamp subseconds values from the correspondent registers */
    return (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).TSSSR;
}
/* *
  * @}
  */
/* * @defgroup RTC_Group9 Tampers configuration functions
 *  @brief   Tampers configuration functions 
 *
@verbatim   
 ===============================================================================
                ##### Tampers configuration functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the select Tamper pin edge.
  * @param  RTC_Tamper: Selected tamper pin.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_Tamper_1: Select Tamper 1.
  *     @arg RTC_Tamper_2: Select Tamper 2.
  *     @arg RTC_Tamper_3: Select Tamper 3.
  * @param  RTC_TamperTrigger: Specifies the trigger on the tamper pin that 
  *                            stimulates tamper event. 
  *   This parameter can be one of the following values:
  *     @arg RTC_TamperTrigger_RisingEdge: Rising Edge of the tamper pin causes tamper event.
  *     @arg RTC_TamperTrigger_FallingEdge: Falling Edge of the tamper pin causes tamper event.
  *     @arg RTC_TamperTrigger_LowLevel: Low Level of the tamper pin causes tamper event.
  *     @arg RTC_TamperTrigger_HighLevel: High Level of the tamper pin causes tamper event.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TamperTriggerConfig(mut RTC_Tamper: uint32_t,
                                                 mut RTC_TamperTrigger:
                                                     uint32_t) {
    /* Check the parameters */
    /* Check if the  active level for Tamper is rising edge (Low level)*/
    if RTC_TamperTrigger == 0i32 as uint32_t {
        /* Configure the RTC_TAFCR register */
        let ref mut fresh25 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh25,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh25
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(RTC_Tamper << 1i32)) as uint32_t as
                                        uint32_t)
    } else {
        /* Configure the RTC_TAFCR register */
        let ref mut fresh26 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh26,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh26
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RTC_Tamper << 1i32)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Enables or Disables the Tamper detection.
  * @param  RTC_Tamper: Selected tamper pin.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_Tamper_1: Select Tamper 1.
  *     @arg RTC_Tamper_2: Select Tamper 2.
  *     @arg RTC_Tamper_3: Select Tamper 3.
  * @param  NewState: new state of the tamper pin.
  *         This parameter can be: ENABLE or DISABLE.                   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TamperCmd(mut RTC_Tamper: uint32_t,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the selected Tamper pin */
        let ref mut fresh27 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh27,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh27
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | RTC_Tamper) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the selected Tamper pin */
        let ref mut fresh28 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh28,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh28
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !RTC_Tamper) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures the Tampers Filter.
  * @param  RTC_TamperFilter: Specifies the tampers filter.
  *   This parameter can be one of the following values:
  *     @arg RTC_TamperFilter_Disable: Tamper filter is disabled.
  *     @arg RTC_TamperFilter_2Sample: Tamper is activated after 2 consecutive 
  *                                    samples at the active level 
  *     @arg RTC_TamperFilter_4Sample: Tamper is activated after 4 consecutive 
  *                                    samples at the active level
  *     @arg RTC_TamperFilter_8Sample: Tamper is activated after 8 consecutive 
  *                                    samples at the active level 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TamperFilterConfig(mut RTC_TamperFilter:
                                                    uint32_t) {
    /* Check the parameters */
    /* Clear TAMPFLT[1:0] bits in the RTC_TAFCR register */
    let ref mut fresh29 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh29,
                                (::core::ptr::read_volatile::<uint32_t>(fresh29
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1800i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Configure the RTC_TAFCR register */
    let ref mut fresh30 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh30,
                                (::core::ptr::read_volatile::<uint32_t>(fresh30
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RTC_TamperFilter) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the Tampers Sampling Frequency.
  * @param  RTC_TamperSamplingFreq: Specifies the tampers Sampling Frequency.
  *   This parameter can be one of the following values:
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div32768: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 32768
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div16384: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 16384
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div8192: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 8192
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div4096: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 4096
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div2048: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 2048
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div1024: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 1024
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div512: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 512  
  *     @arg RTC_TamperSamplingFreq_RTCCLK_Div256: Each of the tamper inputs are sampled
  *                                           with a frequency =  RTCCLK / 256  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TamperSamplingFreqConfig(mut RTC_TamperSamplingFreq:
                                                          uint32_t) {
    /* Check the parameters */
    /* Clear TAMPFREQ[2:0] bits in the RTC_TAFCR register */
    let ref mut fresh31 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh31,
                                (::core::ptr::read_volatile::<uint32_t>(fresh31
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x700i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Configure the RTC_TAFCR register */
    let ref mut fresh32 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh32,
                                (::core::ptr::read_volatile::<uint32_t>(fresh32
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RTC_TamperSamplingFreq)
                                    as uint32_t as uint32_t);
}
/* *
  * @brief  Configures the Tampers Pins input Precharge Duration.
  * @param  RTC_TamperPrechargeDuration: Specifies the Tampers Pins input
  *         Precharge Duration.
  *   This parameter can be one of the following values:
  *     @arg RTC_TamperPrechargeDuration_1RTCCLK: Tamper pins are pre-charged before sampling during 1 RTCCLK cycle
  *     @arg RTC_TamperPrechargeDuration_2RTCCLK: Tamper pins are pre-charged before sampling during 2 RTCCLK cycle
  *     @arg RTC_TamperPrechargeDuration_4RTCCLK: Tamper pins are pre-charged before sampling during 4 RTCCLK cycle    
  *     @arg RTC_TamperPrechargeDuration_8RTCCLK: Tamper pins are pre-charged before sampling during 8 RTCCLK cycle
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TamperPinsPrechargeDuration(mut RTC_TamperPrechargeDuration:
                                                             uint32_t) {
    /* Check the parameters */
    /* Clear TAMPPRCH[1:0] bits in the RTC_TAFCR register */
    let ref mut fresh33 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh33,
                                (::core::ptr::read_volatile::<uint32_t>(fresh33
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x6000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Configure the RTC_TAFCR register */
    let ref mut fresh34 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh34,
                                (::core::ptr::read_volatile::<uint32_t>(fresh34
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     RTC_TamperPrechargeDuration) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Enables or Disables the TimeStamp on Tamper Detection Event.
  * @note   The timestamp is valid even the TSE bit in tamper control register 
  *         is reset.   
  * @param  NewState: new state of the timestamp on tamper event.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TimeStampOnTamperDetectionCmd(mut NewState:
                                                               FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Save timestamp on tamper detection event */
        let ref mut fresh35 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh35,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh35
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Tamper detection does not cause a timestamp to be saved */
        let ref mut fresh36 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh36,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh36
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief  Enables or Disables the Precharge of Tamper pin.
  * @param  NewState: new state of tamper pull up.
  *   This parameter can be: ENABLE or DISABLE.                   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_TamperPullUpCmd(mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable precharge of the selected Tamper pin */
        let ref mut fresh37 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh37,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh37
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable precharge of the selected Tamper pin */
        let ref mut fresh38 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh38,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh38
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000i32 as uint32_t) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup RTC_Group10 Backup Data Registers configuration functions
 *  @brief   Backup Data Registers configuration functions  
 *
@verbatim   
 ===============================================================================
          ##### Backup Data Registers configuration functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Writes a data in a specified RTC Backup data register.
  * @param  RTC_BKP_DR: RTC Backup data Register number.
  *   This parameter can be: RTC_BKP_DRx where x can be from 0 to 15 to 
  *                          specify the register.
  * @param  Data: Data to be written in the specified RTC Backup data register.                     
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WriteBackupRegister(mut RTC_BKP_DR: uint32_t,
                                                 mut Data: uint32_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000i32 as
                                     uint32_t).wrapping_add(0x2800i32 as
                                                                libc::c_uint).wrapping_add(0x50i32
                                                                                               as
                                                                                               libc::c_uint));
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(RTC_BKP_DR.wrapping_mul(4i32
                                                                                            as
                                                                                            libc::c_uint))
                                    as uint32_t as uint32_t);
    /* Write the specified register */
    ::core::ptr::write_volatile(tmp as *mut uint32_t, Data);
}
/* *
  * @brief  Reads data from the specified RTC Backup data Register.
  * @param  RTC_BKP_DR: RTC Backup data Register number.
  *   This parameter can be: RTC_BKP_DRx where x can be from 0 to 15 to 
  *                          specify the register.                   
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ReadBackupRegister(mut RTC_BKP_DR: uint32_t)
 -> uint32_t {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (0x40000000i32 as
                                     uint32_t).wrapping_add(0x2800i32 as
                                                                libc::c_uint).wrapping_add(0x50i32
                                                                                               as
                                                                                               libc::c_uint));
    ::core::ptr::write_volatile(&mut tmp as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmp
                                                                            as
                                                                            *const uint32_t)
                                     as
                                     libc::c_uint).wrapping_add(RTC_BKP_DR.wrapping_mul(4i32
                                                                                            as
                                                                                            libc::c_uint))
                                    as uint32_t as uint32_t);
    /* Read the specified register */
    return *(tmp as *mut uint32_t);
}
/* *
  * @}
  */
/* * @defgroup RTC_Group11 Output Type Config configuration functions
 *  @brief   Output Type Config configuration functions  
 *
@verbatim   
 ===============================================================================
            ##### Output Type Config configuration functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the RTC Output Pin mode. 
  * @param  RTC_OutputType: specifies the RTC Output (PC13) pin mode.
  *   This parameter can be one of the following values:
  *     @arg RTC_OutputType_OpenDrain: RTC Output (PC13) is configured in 
  *                                    Open Drain mode.
  *     @arg RTC_OutputType_PushPull:  RTC Output (PC13) is configured in 
  *                                    Push Pull mode.    
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_OutputTypeConfig(mut RTC_OutputType: uint32_t) {
    /* Check the parameters */
    let ref mut fresh39 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh39,
                                (::core::ptr::read_volatile::<uint32_t>(fresh39
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x40000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    let ref mut fresh40 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR;
    ::core::ptr::write_volatile(fresh40,
                                (::core::ptr::read_volatile::<uint32_t>(fresh40
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | RTC_OutputType) as
                                    uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup RTC_Group12 Shift control synchronisation functions
 *  @brief   Shift control synchronisation functions 
 *
@verbatim   
 ===============================================================================
              ##### Shift control synchronisation functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the Synchronization Shift Control Settings.
  * @note   When REFCKON is set, firmware must not write to Shift control register 
  * @param  RTC_ShiftAdd1S : Select to add or not 1 second to the time Calendar.
  *   This parameter can be one of the following values :
  *     @arg RTC_ShiftAdd1S_Set  : Add one second to the clock calendar. 
  *     @arg RTC_ShiftAdd1S_Reset: No effect.
  * @param  RTC_ShiftSubFS: Select the number of Second Fractions to Substitute.
  *         This parameter can be one any value from 0 to 0x7FFF.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: RTC Shift registers are configured
  *          - ERROR: RTC Shift registers are not configured
*/
#[no_mangle]
pub unsafe extern "C" fn RTC_SynchroShiftConfig(mut RTC_ShiftAdd1S: uint32_t,
                                                mut RTC_ShiftSubFS: uint32_t)
 -> ErrorStatus {
    let mut status: ErrorStatus = ERROR;
    let mut shpfcount: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    /* Check if a Shift is pending*/
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
              as *mut RTC_TypeDef)).ISR & 0x8i32 as uint32_t !=
           RESET as libc::c_int as libc::c_uint {
        /* Wait until the shift is completed*/
        while (*((0x40000000i32 as
                      uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                     *mut RTC_TypeDef)).ISR & 0x8i32 as uint32_t !=
                  RESET as libc::c_int as libc::c_uint &&
                  shpfcount != 0x2000i32 as uint32_t {
            shpfcount = shpfcount.wrapping_add(1)
        }
    }
    /* Check if the Shift pending is completed or if there is no Shift operation at all*/
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
              as *mut RTC_TypeDef)).ISR & 0x8i32 as uint32_t ==
           RESET as libc::c_int as libc::c_uint {
        /* check if the reference clock detection is disabled */
        if (*((0x40000000i32 as
                   uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                  *mut RTC_TypeDef)).CR & 0x10i32 as uint32_t ==
               RESET as libc::c_int as libc::c_uint {
            /* Configure the Shift settings */
            ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                                     uint32_t).wrapping_add(0x2800i32
                                                                                as
                                                                                libc::c_uint)
                                                    as
                                                    *mut RTC_TypeDef)).SHIFTR
                                            as *mut uint32_t,
                                        RTC_ShiftSubFS | RTC_ShiftAdd1S);
            if RTC_WaitForSynchro() as libc::c_uint ==
                   ERROR as libc::c_int as libc::c_uint {
                status = ERROR
            } else { status = SUCCESS }
        } else { status = ERROR }
    } else { status = ERROR }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
    return status;
}
/* *
  * @}
  */
/* * @defgroup RTC_Group13 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions  
 *
@verbatim   
 ===============================================================================
                ##### Interrupts and flags management functions #####
 ===============================================================================  
    [..] All RTC interrupts are connected to the EXTI controller.
         (+) To enable the RTC Alarm interrupt, the following sequence is required:
             (++) Configure and enable the EXTI Line 17 in interrupt mode and select 
                  the rising edge sensitivity using the EXTI_Init() function.
             (++) Configure and enable the RTC_Alarm IRQ channel in the NVIC using 
                  the NVIC_Init() function.
             (++) Configure the RTC to generate RTC alarms (Alarm A and/or Alarm B)
                  using the RTC_SetAlarm() and RTC_AlarmCmd() functions.
         (+) To enable the RTC Wakeup interrupt, the following sequence is required:
             (++) Configure and enable the EXTI Line 20 in interrupt mode and select 
                  the rising edge sensitivity using the EXTI_Init() function.
             (++) Configure and enable the RTC_WKUP IRQ channel in the NVIC using
                  the NVIC_Init() function.
             (++) Configure the RTC to generate the RTC wakeup timer event using the 
                  RTC_WakeUpClockConfig(), RTC_SetWakeUpCounter() and RTC_WakeUpCmd() 
                  functions.
         (+) To enable the RTC Tamper interrupt, the following sequence is required:
             (++) Configure and enable the EXTI Line 19 in interrupt mode and select 
                  the rising edge sensitivity using the EXTI_Init() function.
             (++) Configure and enable the TAMP_STAMP IRQ channel in the NVIC using 
                  the NVIC_Init() function.
             (++) Configure the RTC to detect the RTC tamper event using the 
                  RTC_TamperTriggerConfig() and RTC_TamperCmd() functions.
         (+) To enable the RTC TimeStamp interrupt, the following sequence is required:
             (++) Configure and enable the EXTI Line 19 in interrupt mode and select
                  the rising edge sensitivity using the EXTI_Init() function.
             (++) Configure and enable the TAMP_STAMP IRQ channel in the NVIC using 
                  the NVIC_Init() function.
             (++) Configure the RTC to detect the RTC time-stamp event using the 
                  RTC_TimeStampCmd() functions.

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified RTC interrupts.
  * @param  RTC_IT: specifies the RTC interrupt sources to be enabled or disabled. 
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_TS:  Time Stamp interrupt mask
  *     @arg RTC_IT_WUT:  WakeUp Timer interrupt mask
  *     @arg RTC_IT_ALRB:  Alarm B interrupt mask
  *     @arg RTC_IT_ALRA:  Alarm A interrupt mask
  *     @arg RTC_IT_TAMP: Tamper event interrupt mask
  * @param  NewState: new state of the specified RTC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ITConfig(mut RTC_IT: uint32_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    /* Disable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xcai32 as uint32_t);
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0x53i32 as uint32_t);
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Configure the Interrupts in the RTC_CR register */
        let ref mut fresh41 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh41,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh41
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         RTC_IT & !(0x4i32 as uint32_t)) as
                                        uint32_t as uint32_t);
        /* Configure the Tamper Interrupt in the RTC_TAFCR */
        let ref mut fresh42 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh42,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh42
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         RTC_IT & 0x4i32 as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Configure the Interrupts in the RTC_CR register */
        let ref mut fresh43 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh43,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh43
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(RTC_IT & !(0x4i32 as uint32_t))) as
                                        uint32_t as uint32_t);
        /* Configure the Tamper Interrupt in the RTC_TAFCR */
        let ref mut fresh44 =
            (*((0x40000000i32 as
                    uint32_t).wrapping_add(0x2800i32 as libc::c_uint) as
                   *mut RTC_TypeDef)).TAFCR;
        ::core::ptr::write_volatile(fresh44,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh44
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(RTC_IT & 0x4i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    /* Enable the write protection for RTC registers */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).WPR as
                                    *mut uint32_t, 0xffi32 as uint32_t);
}
/* *
  * @brief  Checks whether the specified RTC flag is set or not.
  * @param  RTC_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg RTC_FLAG_RECALPF: RECALPF event flag
  *     @arg RTC_FLAG_TAMP3F: Tamper 3 event flag
  *     @arg RTC_FLAG_TAMP2F: Tamper 2 event flag   
  *     @arg RTC_FLAG_TAMP1F: Tamper 1 event flag
  *     @arg RTC_FLAG_TSOVF: Time Stamp OverFlow flag
  *     @arg RTC_FLAG_TSF: Time Stamp event flag
  *     @arg RTC_FLAG_WUTF: WakeUp Timer flag
  *     @arg RTC_FLAG_ALRBF: Alarm B flag
  *     @arg RTC_FLAG_ALRAF: Alarm A flag
  *     @arg RTC_FLAG_INITF: Initialization mode flag
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag
  *     @arg RTC_FLAG_INITS: Registers Configured flag
  *     @argRTC_FLAG_SHPF  : Shift operation pending flag.  
  *     @arg RTC_FLAG_WUTWF: WakeUp Timer Write flag
  *     @arg RTC_FLAG_ALRBWF: Alarm B Write flag
  *     @arg RTC_FLAG_ALRAWF: Alarm A write flag
  * @retval The new state of RTC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetFlagStatus(mut RTC_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get all the flags */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).ISR &
            (0x1000i32 as uint32_t | 0x800i32 as uint32_t |
                 0x400i32 as uint32_t | 0x200i32 as uint32_t |
                 0x100i32 as uint32_t | 0x40i32 as uint32_t |
                 0x20i32 as uint32_t | 0x10i32 as uint32_t |
                 0x4i32 as uint32_t | 0x2i32 as uint32_t | 0x1i32 as uint32_t
                 | 0x2000i32 as uint32_t | 0x4000i32 as uint32_t |
                 0x8000i32 as uint32_t | 0x10000i32 as uint32_t |
                 0x8i32 as uint32_t);
    /* Return the status of the flag */
    if tmpreg & RTC_FLAG != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the RTC's pending flags.
  * @param  RTC_FLAG: specifies the RTC flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_FLAG_TAMP3F: Tamper 3 event flag
  *     @arg RTC_FLAG_TAMP2F: Tamper 2 event flag
  *     @arg RTC_FLAG_TAMP1F: Tamper 1 event flag 
  *     @arg RTC_FLAG_TSOVF: Time Stamp Overflow flag 
  *     @arg RTC_FLAG_TSF: Time Stamp event flag
  *     @arg RTC_FLAG_WUTF: WakeUp Timer flag
  *     @arg RTC_FLAG_ALRBF: Alarm B flag
  *     @arg RTC_FLAG_ALRAF: Alarm A flag
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ClearFlag(mut RTC_FLAG: uint32_t) {
    /* Check the parameters */
    /* Clear the Flags in the RTC_ISR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).ISR as
                                    *mut uint32_t,
                                !((RTC_FLAG | 0x80i32 as uint32_t) &
                                      0x1ffffi32 as libc::c_uint) |
                                    (*((0x40000000i32 as
                                            uint32_t).wrapping_add(0x2800i32
                                                                       as
                                                                       libc::c_uint)
                                           as *mut RTC_TypeDef)).ISR &
                                        0x80i32 as uint32_t);
}
/* *
  * @brief  Checks whether the specified RTC interrupt has occurred or not.
  * @param  RTC_IT: specifies the RTC interrupt source to check.
  *   This parameter can be one of the following values:
  *     @arg RTC_IT_TS: Time Stamp interrupt 
  *     @arg RTC_IT_WUT: WakeUp Timer interrupt 
  *     @arg RTC_IT_ALRB: Alarm B interrupt 
  *     @arg RTC_IT_ALRA: Alarm A interrupt 
  *     @arg RTC_IT_TAMP1: Tamper1 event interrupt 
  *     @arg RTC_IT_TAMP2: Tamper2 event interrupt 
  *     @arg RTC_IT_TAMP3: Tamper3 event interrupt
  * @retval The new state of RTC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetITStatus(mut RTC_IT: uint32_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    let mut enablestatus: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the TAMPER Interrupt enable bit and pending bit */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).TAFCR & 0x4i32 as uint32_t;
    /* Get the Interrupt enable Status */
    enablestatus =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).CR & RTC_IT |
            tmpreg & RTC_IT >> (RTC_IT >> 18i32) >> 15i32;
    /* Get the Interrupt pending bit */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2800i32 as libc::c_uint)
               as *mut RTC_TypeDef)).ISR & RTC_IT >> 4i32;
    /* Get the status of the Interrupt */
    if enablestatus != RESET as libc::c_int as uint32_t &&
           tmpreg & 0xffffi32 as libc::c_uint !=
               RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_rtc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the RTC firmware 
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
/* * @addtogroup RTC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  RTC Init structures definition  
  */
/* !< Specifies the RTC Hour Format.
                             This parameter can be a value of @ref RTC_Hour_Formats */
/* !< Specifies the RTC Asynchronous Predivider value.
                             This parameter must be set to a value lower than 0x7F */
/* !< Specifies the RTC Synchronous Predivider value.
                             This parameter must be set to a value lower than 0x1FFF */
/* * 
  * @brief  RTC Time structure definition  
  */
/* !< Specifies the RTC Time Hour.
                        This parameter must be set to a value in the 0-12 range
                        if the RTC_HourFormat_12 is selected or 0-23 range if
                        the RTC_HourFormat_24 is selected. */
/* !< Specifies the RTC Time Minutes.
                        This parameter must be set to a value in the 0-59 range. */
/* !< Specifies the RTC Time Seconds.
                        This parameter must be set to a value in the 0-59 range. */
/* !< Specifies the RTC AM/PM Time.
                        This parameter can be a value of @ref RTC_AM_PM_Definitions */
/* * 
  * @brief  RTC Date structure definition  
  */
/* !< Specifies the RTC Date WeekDay.
                        This parameter can be a value of @ref RTC_WeekDay_Definitions */
/* !< Specifies the RTC Date Month (in BCD format).
                        This parameter can be a value of @ref RTC_Month_Date_Definitions */
/* !< Specifies the RTC Date.
                        This parameter must be set to a value in the 1-31 range. */
/* !< Specifies the RTC Date Year.
                        This parameter must be set to a value in the 0-99 range. */
/* * 
  * @brief  RTC Alarm structure definition        
  */
/* !< Specifies the RTC Alarm Time members. */
/* !< Specifies the RTC Alarm Masks.
                                     This parameter can be a value of @ref RTC_AlarmMask_Definitions */
/* !< Specifies the RTC Alarm is on Date or WeekDay.
                                     This parameter can be a value of @ref RTC_AlarmDateWeekDay_Definitions */
/* !< Specifies the RTC Alarm Date/WeekDay.
                                     If the Alarm Date is selected, this parameter
                                     must be set to a value in the 1-31 range.
                                     If the Alarm WeekDay is selected, this 
                                     parameter can be a value of @ref RTC_WeekDay_Definitions */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup RTC_Exported_Constants
  * @{
  */
/* * @defgroup RTC_Hour_Formats 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Asynchronous_Predivider 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Synchronous_Predivider 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Time_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_AM_PM_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Year_Date_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Month_Date_Definitions 
  * @{
  */
/* Coded in BCD format */
/* *
  * @}
  */
/* * @defgroup RTC_WeekDay_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Alarm_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_AlarmDateWeekDay_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_AlarmMask_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Alarms_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Alarm_Sub_Seconds_Masks_Definitions
  * @{
  */
/* !< All Alarm SS fields are masked. 
                                                                       There is no comparison on sub seconds 
                                                                       for Alarm */
/* !< SS[14:1] are don't care in Alarm 
                                                                       comparison. Only SS[0] is compared. */
/* !< SS[14:2] are don't care in Alarm 
                                                                       comparison. Only SS[1:0] are compared */
/* !< SS[14:3] are don't care in Alarm 
                                                                       comparison. Only SS[2:0] are compared */
/* !< SS[14:4] are don't care in Alarm 
                                                                       comparison. Only SS[3:0] are compared */
/* !< SS[14:5] are don't care in Alarm 
                                                                       comparison. Only SS[4:0] are compared */
/* !< SS[14:6] are don't care in Alarm 
                                                                       comparison. Only SS[5:0] are compared */
/* !< SS[14:7] are don't care in Alarm 
                                                                       comparison. Only SS[6:0] are compared */
/* !< SS[14:8] are don't care in Alarm 
                                                                       comparison. Only SS[7:0] are compared */
/* !< SS[14:9] are don't care in Alarm 
                                                                       comparison. Only SS[8:0] are compared */
/* !< SS[14:10] are don't care in Alarm 
                                                                       comparison. Only SS[9:0] are compared */
/* !< SS[14:11] are don't care in Alarm 
                                                                       comparison. Only SS[10:0] are compared */
/* !< SS[14:12] are don't care in Alarm 
                                                                       comparison.Only SS[11:0] are compared */
/* !< SS[14:13] are don't care in Alarm 
                                                                       comparison. Only SS[12:0] are compared */
/* !< SS[14] is don't care in Alarm 
                                                                       comparison.Only SS[13:0] are compared */
/* !< SS[14:0] are compared and must match 
                                                                       to activate alarm. */
/* *
  * @}
  */
/* * @defgroup RTC_Alarm_Sub_Seconds_Value
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Wakeup_Timer_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Time_Stamp_Edges_definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Output_selection_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Output_Polarity_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Digital_Calibration_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Calib_Output_selection_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Smooth_calib_period_Definitions 
  * @{
  */
/* !<  if RTCCLK = 32768 Hz, Smooth calibation
                                                             period is 32s,  else 2exp20 RTCCLK seconds */
/* !<  if RTCCLK = 32768 Hz, Smooth calibation 
                                                             period is 16s, else 2exp19 RTCCLK seconds */
/* !<  if RTCCLK = 32768 Hz, Smooth calibation 
                                                             period is 8s, else 2exp18 RTCCLK seconds */
/* *
  * @}
  */
/* * @defgroup RTC_Smooth_calib_Plus_pulses_Definitions 
  * @{
  */
/* !<  The number of RTCCLK pulses added  
                                                                during a X -second window = Y - CALM[8:0]. 
                                                                 with Y = 512, 256, 128 when X = 32, 16, 8 */
/* !<  The number of RTCCLK pulses subbstited
                                                                 during a 32-second window =   CALM[8:0]. */
/* *
  * @}
  */
/* * @defgroup RTC_Smooth_calib_Minus_pulses_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_DayLightSaving_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Tamper_Trigger_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Tamper_Filter_Definitions 
  * @{
  */
/* !< Tamper filter is disabled */
/* !< Tamper is activated after 2 
                                                          consecutive samples at the active level */
/* !< Tamper is activated after 4 
                                                          consecutive samples at the active level */
/* !< Tamper is activated after 8 
                                                          consecutive samples at the active leve. */
/* *
  * @}
  */
/* * @defgroup RTC_Tamper_Sampling_Frequencies_Definitions 
  * @{
  */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 32768 */
/* !< Each of the tamper inputs are sampled
                                                                            with a frequency =  RTCCLK / 16384 */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 8192  */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 4096  */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 2048  */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 1024  */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 512   */
/* !< Each of the tamper inputs are sampled
                                                                           with a frequency =  RTCCLK / 256   */
/* *
  * @}
  */
/* * @defgroup RTC_Tamper_Pin_Precharge_Duration_Definitions 
  * @{
  */
/* !< Tamper pins are pre-charged before 
                                                                         sampling during 1 RTCCLK cycle */
/* !< Tamper pins are pre-charged before 
                                                                         sampling during 2 RTCCLK cycles */
/* !< Tamper pins are pre-charged before 
                                                                         sampling during 4 RTCCLK cycles */
/* !< Tamper pins are pre-charged before 
                                                                         sampling during 8 RTCCLK cycles */
/* *
  * @}
  */
/* * @defgroup RTC_Tamper_Pins_Definitions 
  * @{
  */
/* !< Tamper detection enable for 
                                                      input tamper 1 */
/* !< Tamper detection enable for 
                                                      input tamper 2 */
/* !< Tamper detection enable for 
                                                      input tamper 3 */
/* *
  * @}
  */
/* * @defgroup RTC_Output_Type_ALARM_OUT 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Add_1_Second_Parameter_Definitions
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Substract_Fraction_Of_Second_Value
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Backup_Registers_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Input_parameter_format_definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Flags_Definitions 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Interrupts_Definitions 
  * @{
  */
/* Used only to Enable the Tamper Interrupt */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*  Function used to set the RTC configuration to the default reset state *****/
/* Initialization and Configuration functions *********************************/
/* Time and Date configuration functions **************************************/
/* Alarms (Alarm A and Alarm B) configuration functions  **********************/
/* WakeUp Timer configuration functions ***************************************/
/* Daylight Saving configuration functions ************************************/
/* Output pin Configuration function ******************************************/
/* Digital Calibration configuration functions ********************************/
/* TimeStamp configuration functions ******************************************/
/* Tampers configuration functions ********************************************/
/* Backup Data Registers configuration functions ******************************/
/* Output Type Config configuration functions *********************************/
/* RTC_Shift_control_synchonisation_functions *********************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears the RTC's interrupt pending bits.
  * @param  RTC_IT: specifies the RTC interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_TS: Time Stamp interrupt 
  *     @arg RTC_IT_WUT: WakeUp Timer interrupt 
  *     @arg RTC_IT_ALRB: Alarm B interrupt 
  *     @arg RTC_IT_ALRA: Alarm A interrupt 
  *     @arg RTC_IT_TAMP1: Tamper1 event interrupt
  *     @arg RTC_IT_TAMP2: Tamper2 event interrupt
  *     @arg RTC_IT_TAMP3: Tamper3 event interrupt 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ClearITPendingBit(mut RTC_IT: uint32_t) {
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the RTC_ISR Interrupt pending bits mask */
    tmpreg = RTC_IT >> 4i32;
    /* Clear the interrupt pending bits in the RTC_ISR register */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2800i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).ISR as
                                    *mut uint32_t,
                                !((tmpreg | 0x80i32 as uint32_t) &
                                      0xffffi32 as libc::c_uint) |
                                    (*((0x40000000i32 as
                                            uint32_t).wrapping_add(0x2800i32
                                                                       as
                                                                       libc::c_uint)
                                           as *mut RTC_TypeDef)).ISR &
                                        0x80i32 as uint32_t);
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* *
  * @}
  */
/* *
  * @brief  Converts a 2 digit decimal to BCD format.
  * @param  Value: Byte to be converted.
  * @retval Converted byte
  */
unsafe extern "C" fn RTC_ByteToBcd2(mut Value: uint8_t) -> uint8_t {
    let mut bcdhigh: uint8_t = 0i32 as uint8_t;
    while Value as libc::c_int >= 10i32 {
        bcdhigh = bcdhigh.wrapping_add(1);
        Value = (Value as libc::c_int - 10i32) as uint8_t
    }
    return (((bcdhigh as libc::c_int) << 4i32) as uint8_t as libc::c_int |
                Value as libc::c_int) as uint8_t;
}
/* *
  * @brief  Convert from 2 digit BCD to Binary.
  * @param  Value: BCD value to be converted.
  * @retval Converted word
  */
unsafe extern "C" fn RTC_Bcd2ToByte(mut Value: uint8_t) -> uint8_t {
    let mut tmp: uint8_t = 0i32 as uint8_t;
    tmp =
        (((Value as libc::c_int & 0xf0i32 as uint8_t as libc::c_int) as
              uint8_t as libc::c_int >> 0x4i32 as uint8_t as libc::c_int) *
             10i32) as uint8_t;
    return (tmp as libc::c_int +
                (Value as libc::c_int & 0xfi32 as uint8_t as libc::c_int)) as
               uint8_t;
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
