use core;
use libc;
extern "C" {
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
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* *
  * @brief TIM
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
    /* !< TIM capture/compare register 4,      Address offset: 0x5C */
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_TimeBaseInitTypeDef {
    pub TIM_Prescaler: uint16_t,
    pub TIM_CounterMode: uint16_t,
    pub TIM_Period: uint32_t,
    pub TIM_ClockDivision: uint16_t,
    pub TIM_RepetitionCounter: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_OCInitTypeDef {
    pub TIM_OCMode: uint32_t,
    pub TIM_OutputState: uint16_t,
    pub TIM_OutputNState: uint16_t,
    pub TIM_Pulse: uint32_t,
    pub TIM_OCPolarity: uint16_t,
    pub TIM_OCNPolarity: uint16_t,
    pub TIM_OCIdleState: uint16_t,
    pub TIM_OCNIdleState: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_ICInitTypeDef {
    pub TIM_Channel: uint16_t,
    pub TIM_ICPolarity: uint16_t,
    pub TIM_ICSelection: uint16_t,
    pub TIM_ICPrescaler: uint16_t,
    pub TIM_ICFilter: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_BDTRInitTypeDef {
    pub TIM_OSSRState: uint16_t,
    pub TIM_OSSIState: uint16_t,
    pub TIM_LOCKLevel: uint16_t,
    pub TIM_DeadTime: uint16_t,
    pub TIM_Break: uint16_t,
    pub TIM_BreakPolarity: uint16_t,
    pub TIM_AutomaticOutput: uint16_t,
}
/* Private functions ---------------------------------------------------------*/
/* * @defgroup TIM_Private_Functions
  * @{
  */
/* * @defgroup TIM_Group1 TimeBase management functions
 *  @brief   TimeBase management functions 
 *
@verbatim   
 ===============================================================================
                   ##### TimeBase management functions #####
 ===============================================================================  
  
             
    *** TIM Driver: how to use it in Timing(Time base) Mode ***
    ============================================================ 
    [..]
    To use the Timer in Timing(Time base) mode, the following steps are mandatory:
       
    (#) Enable TIM clock using 
        RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function               
    (#) Fill the TIM_TimeBaseInitStruct with the desired parameters.   
    (#) Call TIM_TimeBaseInit(TIMx, &TIM_TimeBaseInitStruct) to configure 
        the Time Base unit
        with the corresponding configuration        
    (#) Enable the NVIC if you need to generate the update interrupt.        
    (#) Enable the corresponding interrupt using the function 
        TIM_ITConfig(TIMx, TIM_IT_Update)      
    (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
    [..]                  
    (@) All other functions can be used separately to modify, if needed,
        a specific feature of the Timer. 

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the TIMx peripheral registers to their default reset values.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @retval None

  */
#[no_mangle]
pub unsafe extern "C" fn TIM_DeInit(mut TIMx: *mut TIM_TypeDef) {
    /* Check the parameters */
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef {
        RCC_APB2PeriphResetCmd(0x800i32 as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x800i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0i32 as libc::c_uint) as
                      *mut TIM_TypeDef {
        RCC_APB1PeriphResetCmd(0x1i32 as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x1i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x400i32 as libc::c_uint) as
                      *mut TIM_TypeDef {
        RCC_APB1PeriphResetCmd(0x2i32 as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x2i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x800i32 as libc::c_uint) as
                      *mut TIM_TypeDef {
        RCC_APB1PeriphResetCmd(0x4i32 as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x4i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x1000i32 as libc::c_uint) as
                      *mut TIM_TypeDef {
        RCC_APB1PeriphResetCmd(0x10i32 as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x10i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x1400i32 as libc::c_uint) as
                      *mut TIM_TypeDef {
        RCC_APB1PeriphResetCmd(0x20i32 as uint32_t, ENABLE);
        RCC_APB1PeriphResetCmd(0x20i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x10000i32 as
                                                  libc::c_uint).wrapping_add(0x3400i32
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef {
        RCC_APB2PeriphResetCmd(0x2000i32 as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x2000i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x10000i32 as
                                                  libc::c_uint).wrapping_add(0x4000i32
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef {
        RCC_APB2PeriphResetCmd(0x10000i32 as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x10000i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x10000i32 as
                                                  libc::c_uint).wrapping_add(0x4400i32
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef {
        RCC_APB2PeriphResetCmd(0x20000i32 as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x20000i32 as uint32_t, DISABLE);
    } else if TIMx ==
                  (0x40000000i32 as
                       uint32_t).wrapping_add(0x10000i32 as
                                                  libc::c_uint).wrapping_add(0x4800i32
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef {
        RCC_APB2PeriphResetCmd(0x40000i32 as uint32_t, ENABLE);
        RCC_APB2PeriphResetCmd(0x40000i32 as uint32_t, DISABLE);
    };
}
/* *
  * @brief  Initializes the TIMx Time Base Unit peripheral according to 
  *         the specified parameters in the TIM_TimeBaseInitStruct.
  * @param  TIMx: where x can be  1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_TimeBaseInitStruct: pointer to a TIM_TimeBaseInitTypeDef structure
  *         that contains the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_TimeBaseInit(mut TIMx: *mut TIM_TypeDef,
                                          mut TIM_TimeBaseInitStruct:
                                              *mut TIM_TimeBaseInitTypeDef) {
    let mut tmpcr1: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    tmpcr1 = (*TIMx).CR1;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as uint32_t).wrapping_add(0i32 as libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x400i32 as libc::c_uint) as
                   *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x800i32 as libc::c_uint) as
                   *mut TIM_TypeDef {
        /* Select the Counter Mode */
        tmpcr1 =
            (tmpcr1 as libc::c_int &
                 !(0x10i32 as uint16_t as libc::c_int |
                       0x60i32 as uint16_t as libc::c_int) as uint16_t as
                     libc::c_int) as uint16_t;
        tmpcr1 =
            (tmpcr1 as libc::c_uint |
                 (*TIM_TimeBaseInitStruct).TIM_CounterMode as uint32_t) as
                uint16_t
    }
    if TIMx !=
           (0x40000000i32 as uint32_t).wrapping_add(0x1000i32 as libc::c_uint)
               as *mut TIM_TypeDef &&
           TIMx !=
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x1400i32 as libc::c_uint) as
                   *mut TIM_TypeDef {
        /* Set the clock division */
        tmpcr1 =
            (tmpcr1 as libc::c_int &
                 !(0x300i32 as uint16_t as libc::c_int) as uint16_t as
                     libc::c_int) as uint16_t;
        tmpcr1 =
            (tmpcr1 as libc::c_uint |
                 (*TIM_TimeBaseInitStruct).TIM_ClockDivision as uint32_t) as
                uint16_t
    }
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t, tmpcr1);
    /* Set the Autoreload value */
    ::core::ptr::write_volatile(&mut (*TIMx).ARR as *mut uint32_t,
                                (*TIM_TimeBaseInitStruct).TIM_Period);
    /* Set the Prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).PSC as *mut uint16_t,
                                (*TIM_TimeBaseInitStruct).TIM_Prescaler);
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x4000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x4400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x4800i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Set the Repetition Counter value */
        ::core::ptr::write_volatile(&mut (*TIMx).RCR as *mut uint16_t,
                                    (*TIM_TimeBaseInitStruct).TIM_RepetitionCounter)
    }
    /* Generate an update event to reload the Prescaler 
     and the repetition counter(only for TIM1 and TIM8) value immediatly */
    ::core::ptr::write_volatile(&mut (*TIMx).EGR as *mut uint32_t,
                                0x1i32 as uint16_t as uint32_t);
}
/* *
  * @brief  Fills each TIM_TimeBaseInitStruct member with its default value.
  * @param  TIM_TimeBaseInitStruct : pointer to a TIM_TimeBaseInitTypeDef
  *         structure which will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_TimeBaseStructInit(mut TIM_TimeBaseInitStruct:
                                                    *mut TIM_TimeBaseInitTypeDef) {
    /* Set the default configuration */
    (*TIM_TimeBaseInitStruct).TIM_Period = 0xffffffffu32;
    (*TIM_TimeBaseInitStruct).TIM_Prescaler = 0i32 as uint16_t;
    (*TIM_TimeBaseInitStruct).TIM_ClockDivision = 0i32 as uint16_t;
    (*TIM_TimeBaseInitStruct).TIM_CounterMode = 0i32 as uint16_t;
    (*TIM_TimeBaseInitStruct).TIM_RepetitionCounter = 0i32 as uint16_t;
}
/* *
  * @brief  Configures the TIMx Prescaler.
  * @param  TIMx: where x can be  1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  Prescaler: specifies the Prescaler Register value
  * @param  TIM_PSCReloadMode: specifies the TIM Prescaler Reload mode
  *          This parameter can be one of the following values:
  *            @arg TIM_PSCReloadMode_Update: The Prescaler is loaded at the update event.
  *            @arg TIM_PSCReloadMode_Immediate: The Prescaler is loaded immediatly.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_PrescalerConfig(mut TIMx: *mut TIM_TypeDef,
                                             mut Prescaler: uint16_t,
                                             mut TIM_PSCReloadMode:
                                                 uint16_t) {
    /* Check the parameters */
    /* Set the Prescaler value */
    ::core::ptr::write_volatile(&mut (*TIMx).PSC as *mut uint16_t, Prescaler);
    /* Set or reset the UG Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).EGR as *mut uint32_t,
                                TIM_PSCReloadMode as uint32_t);
}
/* *
  * @brief  Specifies the TIMx Counter Mode to be used.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_CounterMode: specifies the Counter Mode to be used
  *          This parameter can be one of the following values:
  *            @arg TIM_CounterMode_Up: TIM Up Counting Mode
  *            @arg TIM_CounterMode_Down: TIM Down Counting Mode
  *            @arg TIM_CounterMode_CenterAligned1: TIM Center Aligned Mode1
  *            @arg TIM_CounterMode_CenterAligned2: TIM Center Aligned Mode2
  *            @arg TIM_CounterMode_CenterAligned3: TIM Center Aligned Mode3
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_CounterModeConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_CounterMode:
                                                   uint16_t) {
    let mut tmpcr1: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    tmpcr1 = (*TIMx).CR1;
    /* Reset the CMS and DIR Bits */
    tmpcr1 =
        (tmpcr1 as libc::c_int &
             !(0x10i32 as uint16_t as libc::c_int |
                   0x60i32 as uint16_t as libc::c_int) as uint16_t as
                 libc::c_int) as uint16_t;
    /* Set the Counter Mode */
    tmpcr1 =
        (tmpcr1 as libc::c_int | TIM_CounterMode as libc::c_int) as uint16_t;
    /* Write to TIMx CR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t, tmpcr1);
}
/* *
  * @brief  Sets the TIMx Counter Register value
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  Counter: specifies the Counter register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCounter(mut TIMx: *mut TIM_TypeDef,
                                        mut Counter: uint32_t) {
    /* Check the parameters */
    /* Set the Counter Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CNT as *mut uint32_t, Counter);
}
/* *
  * @brief  Sets the TIMx Autoreload Register value
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  Autoreload: specifies the Autoreload register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetAutoreload(mut TIMx: *mut TIM_TypeDef,
                                           mut Autoreload: uint32_t) {
    /* Check the parameters */
    /* Set the Autoreload Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).ARR as *mut uint32_t,
                                Autoreload);
}
/* *
  * @brief  Gets the TIMx Counter value.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @retval Counter Register value
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetCounter(mut TIMx: *mut TIM_TypeDef)
 -> uint32_t {
    /* Check the parameters */
    /* Get the Counter Register value */
    return (*TIMx).CNT;
}
/* *
  * @brief  Gets the TIMx Prescaler value.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @retval Prescaler Register value.
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetPrescaler(mut TIMx: *mut TIM_TypeDef)
 -> uint16_t {
    /* Check the parameters */
    /* Get the Prescaler Register value */
    return (*TIMx).PSC;
}
/* *
  * @brief  Enables or Disables the TIMx Update event.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  NewState: new state of the TIMx UDIS bit
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_UpdateDisableConfig(mut TIMx: *mut TIM_TypeDef,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the Update Disable Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x2i32 as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    } else {
        /* Reset the Update Disable Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x2i32 as uint16_t as libc::c_int)
                                             as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Configures the TIMx Update Request Interrupt source.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_UpdateSource: specifies the Update source.
  *          This parameter can be one of the following values:
  *            @arg TIM_UpdateSource_Regular: Source of update is the counter
  *                 overflow/underflow or the setting of UG bit, or an update
  *                 generation through the slave mode controller.
  *            @arg TIM_UpdateSource_Global: Source of update is counter overflow/underflow.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_UpdateRequestConfig(mut TIMx: *mut TIM_TypeDef,
                                                 mut TIM_UpdateSource:
                                                     uint16_t) {
    /* Check the parameters */
    if TIM_UpdateSource as libc::c_int != 0i32 as uint16_t as libc::c_int {
        /* Set the URS Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x4i32 as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    } else {
        /* Reset the URS Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x4i32 as uint16_t as libc::c_int)
                                             as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Sets or resets the update interrupt flag (UIF)status bit Remapping.
  *         when sets, reading TIMx_CNT register returns UIF bit instead of CNT[31]  
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  NewState: new state of the UIFREMAP bit.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_UIFRemap(mut TIMx: *mut TIM_TypeDef,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the TIM Counter */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x800i32 as uint16_t as libc::c_int)
                                        as uint16_t as uint16_t)
    } else {
        /* Disable the TIM Counter */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x800i32 as uint16_t as
                                               libc::c_int) as uint16_t as
                                             libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enables or disables TIMx peripheral Preload register on ARR.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  NewState: new state of the TIMx peripheral Preload register
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ARRPreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the ARR Preload Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x80i32 as uint16_t as libc::c_int)
                                        as uint16_t as uint16_t)
    } else {
        /* Reset the ARR Preload Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x80i32 as uint16_t as libc::c_int)
                                             as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @brief  Selects the TIMx's One Pulse Mode.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6 ,7 ,8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OPMode: specifies the OPM Mode to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_OPMode_Single
  *            @arg TIM_OPMode_Repetitive
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectOnePulseMode(mut TIMx: *mut TIM_TypeDef,
                                                mut TIM_OPMode: uint16_t) {
    /* Check the parameters */
    /* Reset the OPM Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x8i32 as uint16_t as libc::c_int) as
                                         uint16_t as libc::c_int) as uint16_t
                                    as uint16_t);
    /* Configure the OPM Mode */
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     TIM_OPMode as libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Sets the TIMx Clock Division value.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17, to select the TIM peripheral.
  * @param  TIM_CKD: specifies the clock division value.
  *          This parameter can be one of the following value:
  *            @arg TIM_CKD_DIV1: TDTS = Tck_tim
  *            @arg TIM_CKD_DIV2: TDTS = 2*Tck_tim
  *            @arg TIM_CKD_DIV4: TDTS = 4*Tck_tim
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetClockDivision(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_CKD: uint16_t) {
    /* Check the parameters */
    /* Reset the CKD Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x300i32 as uint16_t as libc::c_int) as
                                         uint16_t as libc::c_int) as uint16_t
                                    as uint16_t);
    /* Set the CKD value */
    ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int | TIM_CKD as libc::c_int)
                                    as uint16_t as uint16_t);
}
/* *
  * @brief  Enables or disables the specified TIM peripheral.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select 
  *        the TIMx peripheral.
  * @param  NewState: new state of the TIMx peripheral.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_Cmd(mut TIMx: *mut TIM_TypeDef,
                                 mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the TIM Counter */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         0x1i32 as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    } else {
        /* Disable the TIM Counter */
        ::core::ptr::write_volatile(&mut (*TIMx).CR1 as *mut uint16_t,
                                    (::core::ptr::read_volatile::<uint16_t>(&(*TIMx).CR1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(0x1i32 as uint16_t as libc::c_int)
                                             as uint16_t as libc::c_int) as
                                        uint16_t as uint16_t)
    };
}
/* *
  * @}
  */
/* * @defgroup TIM_Group2 Output Compare management functions
 *  @brief    Output Compare management functions 
 *
@verbatim   
 ===============================================================================
                ##### Output Compare management functions #####
 ===============================================================================  
       
  *** TIM Driver: how to use it in Output Compare Mode ***
  ======================================================== 
  [..] 
  To use the Timer in Output Compare mode, the following steps are mandatory:
       
       (#) Enable TIM clock using RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function
       
       (#) Configure the TIM pins by configuring the corresponding GPIO pins
       
       (#) Configure the Time base unit as described in the first part of this driver, 
           if needed, else the Timer will run with the default configuration:
           (++) Autoreload value = 0xFFFF
           (++) Prescaler value = 0x0000
           (++) Counter mode = Up counting
           (++) Clock Division = TIM_CKD_DIV1   
       (#) Fill the TIM_OCInitStruct with the desired parameters including:
           (++) The TIM Output Compare mode: TIM_OCMode
           (++) TIM Output State: TIM_OutputState
           (++) TIM Pulse value: TIM_Pulse
           (++) TIM Output Compare Polarity : TIM_OCPolarity
       
       (#) Call TIM_OCxInit(TIMx, &TIM_OCInitStruct) to configure the desired channel with the 
           corresponding configuration
       
       (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
 [..]      
       (@) All other functions can be used separately to modify, if needed,
           a specific feature of the Timer. 
          
       (@) In case of PWM mode, this function is mandatory:
           TIM_OCxPreloadConfig(TIMx, TIM_OCPreload_ENABLE); 
              
       (@) If the corresponding interrupt or DMA request are needed, the user should:
                (#@) Enable the NVIC (or the DMA) to use the TIM interrupts (or DMA requests). 
                (#@) Enable the corresponding interrupt (or DMA request) using the function 
                     TIM_ITConfig(TIMx, TIM_IT_CCx) (or TIM_DMA_Cmd(TIMx, TIM_DMA_CCx))   

@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIMx Channel1 according to the specified parameters in
  *         the TIM_OCInitStruct.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17, to select the TIM peripheral.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC1Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_OCInitStruct:
                                         *mut TIM_OCInitTypeDef) {
    let mut tmpccmrx: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmpcr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the Channel 1: Reset the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x1i32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmrx = (*TIMx).CCMR1;
    /* Reset the Output Compare Mode Bits */
    tmpccmrx &= !(0x10070i32 as uint32_t);
    tmpccmrx &= !(0x3i32 as uint32_t);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*TIM_OCInitStruct).TIM_OCMode;
    /* Reset the Output Polarity level */
    tmpccer &= !(0x2i32 as uint32_t);
    /* Set the Output Compare Polarity */
    tmpccer |= (*TIM_OCInitStruct).TIM_OCPolarity as libc::c_uint;
    /* Set the Output State */
    tmpccer |= (*TIM_OCInitStruct).TIM_OutputState as libc::c_uint;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x4000i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x4400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x4800i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output N Polarity level */
        tmpccer &= !(0x8i32 as uint32_t);
        /* Set the Output N Polarity */
        tmpccer |= (*TIM_OCInitStruct).TIM_OCNPolarity as libc::c_uint;
        /* Reset the Output N State */
        tmpccer &= !(0x4i32 as uint32_t);
        /* Set the Output N State */
        tmpccer |= (*TIM_OCInitStruct).TIM_OutputNState as libc::c_uint;
        /* Reset the Output Compare and Output Compare N IDLE State */
        tmpcr2 &= !(0x100i32 as uint32_t);
        tmpcr2 &= !(0x200i32 as uint32_t);
        /* Set the Output Idle state */
        tmpcr2 |= (*TIM_OCInitStruct).TIM_OCIdleState as libc::c_uint;
        /* Set the Output N Idle state */
        tmpcr2 |= (*TIM_OCInitStruct).TIM_OCNIdleState as libc::c_uint
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR1 as *mut uint32_t,
                                (*TIM_OCInitStruct).TIM_Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Initializes the TIMx Channel2 according to the specified parameters 
  *         in the TIM_OCInitStruct.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM peripheral.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC2Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_OCInitStruct:
                                         *mut TIM_OCInitTypeDef) {
    let mut tmpccmrx: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmpcr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x10i32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR1 register value */
    tmpccmrx = (*TIMx).CCMR1;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !(0x1007000i32 as uint32_t);
    tmpccmrx &= !(0x300i32 as uint32_t);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*TIM_OCInitStruct).TIM_OCMode << 8i32;
    /* Reset the Output Polarity level */
    tmpccer &= !(0x20i32 as uint32_t);
    /* Set the Output Compare Polarity */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OCPolarity as uint32_t) << 4i32;
    /* Set the Output State */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OutputState as uint32_t) << 4i32;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output N Polarity level */
        tmpccer &= !(0x80i32 as uint32_t);
        /* Set the Output N Polarity */
        tmpccer |= ((*TIM_OCInitStruct).TIM_OCNPolarity as uint32_t) << 4i32;
        /* Reset the Output N State */
        tmpccer &= !(0x40i32 as uint32_t);
        /* Set the Output N State */
        tmpccer |= ((*TIM_OCInitStruct).TIM_OutputNState as uint32_t) << 4i32;
        /* Reset the Output Compare and Output Compare N IDLE State */
        tmpcr2 &= !(0x400i32 as uint32_t);
        tmpcr2 &= !(0x800i32 as uint32_t);
        /* Set the Output Idle state */
        tmpcr2 |= ((*TIM_OCInitStruct).TIM_OCIdleState as uint32_t) << 2i32;
        /* Set the Output N Idle state */
        tmpcr2 |= ((*TIM_OCInitStruct).TIM_OCNIdleState as uint32_t) << 2i32
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR2 as *mut uint32_t,
                                (*TIM_OCInitStruct).TIM_Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Initializes the TIMx Channel3 according to the specified parameters
  *         in the TIM_OCInitStruct.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC3Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_OCInitStruct:
                                         *mut TIM_OCInitTypeDef) {
    let mut tmpccmrx: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmpcr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the Channel 3: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x100i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR2 register value */
    tmpccmrx = (*TIMx).CCMR2;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !(0x70i32 as uint32_t);
    tmpccmrx &= !(0x3i32 as uint32_t);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*TIM_OCInitStruct).TIM_OCMode;
    /* Reset the Output Polarity level */
    tmpccer &= !(0x200i32 as uint32_t);
    /* Set the Output Compare Polarity */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OCPolarity as uint32_t) << 8i32;
    /* Set the Output State */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OutputState as uint32_t) << 8i32;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output N Polarity level */
        tmpccer &= !(0x800i32 as uint32_t);
        /* Set the Output N Polarity */
        tmpccer |= ((*TIM_OCInitStruct).TIM_OCNPolarity as uint32_t) << 8i32;
        /* Reset the Output N State */
        tmpccer &= !(0x400i32 as uint32_t);
        /* Set the Output N State */
        tmpccer |= ((*TIM_OCInitStruct).TIM_OutputNState as uint32_t) << 8i32;
        /* Reset the Output Compare and Output Compare N IDLE State */
        tmpcr2 &= !(0x1000i32 as uint32_t);
        tmpcr2 &= !(0x2000i32 as uint32_t);
        /* Set the Output Idle state */
        tmpcr2 |= ((*TIM_OCInitStruct).TIM_OCIdleState as uint32_t) << 4i32;
        /* Set the Output N Idle state */
        tmpcr2 |= ((*TIM_OCInitStruct).TIM_OCNIdleState as uint32_t) << 4i32
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR3 as *mut uint32_t,
                                (*TIM_OCInitStruct).TIM_Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Initializes the TIMx Channel4 according to the specified parameters
  *         in the TIM_OCInitStruct.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC4Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_OCInitStruct:
                                         *mut TIM_OCInitTypeDef) {
    let mut tmpccmrx: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmpcr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the Channel 4: Reset the CC4E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1000i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR2 register value */
    tmpccmrx = (*TIMx).CCMR2;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !(0x7000i32 as uint32_t);
    tmpccmrx &= !(0x300i32 as uint32_t);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*TIM_OCInitStruct).TIM_OCMode << 8i32;
    /* Reset the Output Polarity level */
    tmpccer &= !(0x2000i32 as uint32_t);
    /* Set the Output Compare Polarity */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OCPolarity as uint32_t) << 12i32;
    /* Set the Output State */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OutputState as uint32_t) << 12i32;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output Compare IDLE State */
        tmpcr2 &= !(0x4000i32 as uint32_t);
        /* Set the Output Idle state */
        tmpcr2 |= ((*TIM_OCInitStruct).TIM_OCIdleState as uint32_t) << 6i32
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR4 as *mut uint32_t,
                                (*TIM_OCInitStruct).TIM_Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Initializes the TIMx Channel5 according to the specified parameters
  *         in the TIM_OCInitStruct.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC5Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_OCInitStruct:
                                         *mut TIM_OCInitTypeDef) {
    let mut tmpccmrx: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmpcr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the Channel 5: Reset the CC5E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x10000i32 as uint32_t)) as uint32_t as
                                    uint32_t); /* to be verified*/
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR3 register value */
    tmpccmrx = (*TIMx).CCMR3;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !(0x70i32 as uint32_t);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*TIM_OCInitStruct).TIM_OCMode;
    /* Reset the Output Polarity level */
    tmpccer &= !(0x20000i32 as uint32_t);
    /* Set the Output Compare Polarity */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OCPolarity as uint32_t) << 16i32;
    /* Set the Output State */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OutputState as uint32_t) << 16i32;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output Compare IDLE State */
        tmpcr2 &= !(0x10000i32 as uint32_t);
        /* Set the Output Idle state */
        tmpcr2 |= ((*TIM_OCInitStruct).TIM_OCIdleState as uint32_t) << 16i32
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                (*TIM_OCInitStruct).TIM_Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Initializes the TIMx Channel6 according to the specified parameters
  *         in the TIM_OCInitStruct.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC6Init(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_OCInitStruct:
                                         *mut TIM_OCInitTypeDef) {
    let mut tmpccmrx: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmpcr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Disable the Channel 5: Reset the CC5E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x100000i32 as uint32_t)) as uint32_t
                                    as uint32_t); /* to be verified*/
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER;
    /* Get the TIMx CR2 register value */
    tmpcr2 = (*TIMx).CR2;
    /* Get the TIMx CCMR3 register value */
    tmpccmrx = (*TIMx).CCMR3;
    /* Reset the Output Compare mode and Capture/Compare selection Bits */
    tmpccmrx &= !(0x7000i32 as uint32_t);
    /* Select the Output Compare Mode */
    tmpccmrx |= (*TIM_OCInitStruct).TIM_OCMode << 8i32;
    /* Reset the Output Polarity level */
    tmpccer &= !(0x200000i32 as uint32_t);
    /* Set the Output Compare Polarity */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OCPolarity as uint32_t) << 20i32;
    /* Set the Output State */
    tmpccer |= ((*TIM_OCInitStruct).TIM_OutputState as uint32_t) << 20i32;
    if TIMx ==
           (0x40000000i32 as
                uint32_t).wrapping_add(0x10000i32 as
                                           libc::c_uint).wrapping_add(0x2c00i32
                                                                          as
                                                                          libc::c_uint)
               as *mut TIM_TypeDef ||
           TIMx ==
               (0x40000000i32 as
                    uint32_t).wrapping_add(0x10000i32 as
                                               libc::c_uint).wrapping_add(0x3400i32
                                                                              as
                                                                              libc::c_uint)
                   as *mut TIM_TypeDef {
        /* Reset the Output Compare IDLE State */
        tmpcr2 &= !(0x20000i32 as uint32_t);
        /* Set the Output Idle state */
        tmpcr2 |=
            (((*TIM_OCInitStruct).TIM_OCIdleState as libc::c_int) << 18i32) as
                uint16_t as libc::c_uint
    }
    /* Write to TIMx CR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t, tmpcr2);
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmrx);
    /* Set the Capture Compare Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR6 as *mut uint32_t,
                                (*TIM_OCInitStruct).TIM_Pulse);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Selects the TIM Group Channel 5 and Channel 1, 
            OC1REFC is the logical AND of OC1REFC and OC5REF.
  * @param  TIMx: where x can be  1 or 8 to select the TIMx peripheral
  * @param  NewState: new state of the Commutation event.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectGC5C1(mut TIMx: *mut TIM_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the GC5C1 Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCR5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x20000000i32 as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the GC5C1 Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCR5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x20000000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the TIM Group Channel 5 and Channel 2, 
            OC2REFC is the logical AND of OC2REFC and OC5REF.
  * @param  TIMx: where x can be  1 or 8 to select the TIMx peripheral
  * @param  NewState: new state of the Commutation event.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectGC5C2(mut TIMx: *mut TIM_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the GC5C2 Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCR5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x40000000i32 as uint32_t) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the GC5C2 Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCR5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x40000000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the TIM Group Channel 5 and Channel 3, 
            OC3REFC is the logical AND of OC3REFC and OC5REF.
  * @param  TIMx: where x can be  1 or 8 to select the TIMx peripheral
  * @param  NewState: new state of the Commutation event.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectGC5C3(mut TIMx: *mut TIM_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the GC5C3 Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCR5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x80000000u32) as
                                        uint32_t as uint32_t)
    } else {
        /* Reset the GC5C3 Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCR5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !0x80000000u32) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Fills each TIM_OCInitStruct member with its default value.
  * @param  TIM_OCInitStruct: pointer to a TIM_OCInitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OCStructInit(mut TIM_OCInitStruct:
                                              *mut TIM_OCInitTypeDef) {
    /* Set the default configuration */
    (*TIM_OCInitStruct).TIM_OCMode = 0i32 as uint32_t;
    (*TIM_OCInitStruct).TIM_OutputState = 0i32 as uint16_t;
    (*TIM_OCInitStruct).TIM_OutputNState = 0i32 as uint16_t;
    (*TIM_OCInitStruct).TIM_Pulse = 0i32 as uint32_t;
    (*TIM_OCInitStruct).TIM_OCPolarity = 0i32 as uint16_t;
    (*TIM_OCInitStruct).TIM_OCNPolarity = 0i32 as uint16_t;
    (*TIM_OCInitStruct).TIM_OCIdleState = 0i32 as uint16_t;
    (*TIM_OCInitStruct).TIM_OCNIdleState = 0i32 as uint16_t;
}
/* *
  * @brief  Selects the TIM Output Compare Mode.
  * @note   This function disables the selected channel before changing the Output
  *         Compare Mode. If needed, user has to enable this channel using
  *         TIM_CCxCmd() and TIM_CCxNCmd() functions.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  *            @arg TIM_Channel_4: TIM Channel 4
  * @param  TIM_OCMode: specifies the TIM Output Compare Mode.
  *           This parameter can be one of the following values:
  *            @arg TIM_OCMode_Timing
  *            @arg TIM_OCMode_Active
  *            @arg TIM_OCMode_Toggle
  *            @arg TIM_OCMode_PWM1
  *            @arg TIM_OCMode_PWM2
  *            @arg TIM_ForcedAction_Active
  *            @arg TIM_ForcedAction_InActive
  *            @arg TIM_OCMode_Retrigerrable_OPM1
  *            @arg TIM_OCMode_Retrigerrable_OPM2
  *            @arg TIM_OCMode_Combined_PWM1
  *            @arg TIM_OCMode_Combined_PWM2
  *            @arg TIM_OCMode_Asymmetric_PWM1
  *            @arg TIM_OCMode_Asymmetric_PWM2            
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectOCxM(mut TIMx: *mut TIM_TypeDef,
                                        mut TIM_Channel: uint16_t,
                                        mut TIM_OCMode: uint32_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    let mut tmp1: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    tmp = TIMx as uint32_t;
    tmp =
        (tmp as
             libc::c_uint).wrapping_add(0x18i32 as uint16_t as libc::c_uint)
            as uint32_t as uint32_t;
    tmp1 =
        ((0x1i32 as uint16_t as libc::c_int) << TIM_Channel as libc::c_int) as
            uint16_t;
    /* Disable the Channel: Reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(tmp1 as libc::c_int) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    if TIM_Channel as libc::c_int == 0i32 as uint16_t as libc::c_int ||
           TIM_Channel as libc::c_int == 0x8i32 as uint16_t as libc::c_int {
        tmp =
            (tmp as
                 libc::c_uint).wrapping_add((TIM_Channel as libc::c_int >>
                                                 1i32) as libc::c_uint) as
                uint32_t as uint32_t;
        /* Reset the OCxM bits in the CCMRx register */
        let ref mut fresh0 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & 0xfffeff8fu32) as
                                        uint32_t as uint32_t);
        /* Configure the OCxM bits in the CCMRx register */
        let ref mut fresh1 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | TIM_OCMode) as
                                        uint32_t as uint32_t)
    } else {
        tmp =
            (tmp as
                 libc::c_uint).wrapping_add((TIM_Channel as
                                                 libc::c_uint).wrapping_sub(4i32
                                                                                as
                                                                                uint32_t)
                                                >> 1i32 as uint32_t) as
                uint32_t as uint32_t;
        /* Reset the OCxM bits in the CCMRx register */
        let ref mut fresh2 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & 0xfeff8fffu32) as
                                        uint32_t as uint32_t);
        /* Configure the OCxM bits in the CCMRx register */
        let ref mut fresh3 = *(tmp as *mut uint32_t);
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | TIM_OCMode << 8i32)
                                        as uint32_t as uint32_t)
    };
}
/* *
  * @brief  Sets the TIMx Capture Compare1 Register value
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  Compare1: specifies the Capture Compare1 register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCompare1(mut TIMx: *mut TIM_TypeDef,
                                         mut Compare1: uint32_t) {
    /* Check the parameters */
    /* Set the Capture Compare1 Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR1 as *mut uint32_t, Compare1);
}
/* *
  * @brief  Sets the TIMx Capture Compare2 Register value
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  Compare2: specifies the Capture Compare2 register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCompare2(mut TIMx: *mut TIM_TypeDef,
                                         mut Compare2: uint32_t) {
    /* Check the parameters */
    /* Set the Capture Compare2 Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR2 as *mut uint32_t, Compare2);
}
/* *
  * @brief  Sets the TIMx Capture Compare3 Register value
  * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * @param  Compare3: specifies the Capture Compare3 register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCompare3(mut TIMx: *mut TIM_TypeDef,
                                         mut Compare3: uint32_t) {
    /* Check the parameters */
    /* Set the Capture Compare3 Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR3 as *mut uint32_t, Compare3);
}
/* *
  * @brief  Sets the TIMx Capture Compare4 Register value
  * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * @param  Compare4: specifies the Capture Compare4 register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCompare4(mut TIMx: *mut TIM_TypeDef,
                                         mut Compare4: uint32_t) {
    /* Check the parameters */
    /* Set the Capture Compare4 Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR4 as *mut uint32_t, Compare4);
}
/* *
  * @brief  Sets the TIMx Capture Compare5 Register value
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  Compare5: specifies the Capture Compare5 register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCompare5(mut TIMx: *mut TIM_TypeDef,
                                         mut Compare5: uint32_t) {
    /* Check the parameters */
    /* Set the Capture Compare5 Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR5 as *mut uint32_t, Compare5);
}
/* *
  * @brief  Sets the TIMx Capture Compare6 Register value
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  Compare6: specifies the Capture Compare5 register new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetCompare6(mut TIMx: *mut TIM_TypeDef,
                                         mut Compare6: uint32_t) {
    /* Check the parameters */
    /* Set the Capture Compare6 Register value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCR6 as *mut uint32_t, Compare6);
}
/* *
  * @brief  Forces the TIMx output 1 waveform to active or inactive level.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
  *          This parameter can be one of the following values:
  *            @arg TIM_ForcedAction_Active: Force active level on OC1REF
  *            @arg TIM_ForcedAction_InActive: Force inactive level on OC1REF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ForcedOC1Config(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ForcedAction: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC1M Bits */
    tmpccmr1 &= !(0x10070i32 as uint32_t);
    /* Configure The Forced output Mode */
    tmpccmr1 |= TIM_ForcedAction as libc::c_uint;
    /* Write to TIMx CCMR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Forces the TIMx output 2 waveform to active or inactive level.
  * @param  TIMx: where x can be   1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
  *          This parameter can be one of the following values:
  *            @arg TIM_ForcedAction_Active: Force active level on OC2REF
  *            @arg TIM_ForcedAction_InActive: Force inactive level on OC2REF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ForcedOC2Config(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ForcedAction: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC2M Bits */
    tmpccmr1 &= !(0x1007000i32 as uint32_t);
    /* Configure The Forced output Mode */
    tmpccmr1 |= (TIM_ForcedAction as uint32_t) << 8i32;
    /* Write to TIMx CCMR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Forces the TIMx output 3 waveform to active or inactive level.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
  *          This parameter can be one of the following values:
  *            @arg TIM_ForcedAction_Active: Force active level on OC3REF
  *            @arg TIM_ForcedAction_InActive: Force inactive level on OC3REF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ForcedOC3Config(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ForcedAction: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC1M Bits */
    tmpccmr2 &= !(0x70i32 as uint32_t);
    /* Configure The Forced output Mode */
    tmpccmr2 |= TIM_ForcedAction as libc::c_uint;
    /* Write to TIMx CCMR2 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Forces the TIMx output 4 waveform to active or inactive level.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
  *          This parameter can be one of the following values:
  *            @arg TIM_ForcedAction_Active: Force active level on OC4REF
  *            @arg TIM_ForcedAction_InActive: Force inactive level on OC4REF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ForcedOC4Config(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ForcedAction: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC2M Bits */
    tmpccmr2 &= !(0x7000i32 as uint32_t);
    /* Configure The Forced output Mode */
    tmpccmr2 |= (TIM_ForcedAction as uint32_t) << 8i32;
    /* Write to TIMx CCMR2 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Forces the TIMx output 5 waveform to active or inactive level.
  * @param  TIMx: where x can be  1 or 8 to select the TIM peripheral.
  * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
  *          This parameter can be one of the following values:
  *            @arg TIM_ForcedAction_Active: Force active level on OC5REF
  *            @arg TIM_ForcedAction_InActive: Force inactive level on OC5REF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ForcedOC5Config(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ForcedAction: uint16_t) {
    let mut tmpccmr3: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Reset the OC5M Bits */
    tmpccmr3 &= !(0x70i32 as uint32_t);
    /* Configure The Forced output Mode */
    tmpccmr3 |= TIM_ForcedAction as uint32_t;
    /* Write to TIMx CCMR3 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
}
/* *
  * @brief  Forces the TIMx output 6 waveform to active or inactive level.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_ForcedAction: specifies the forced Action to be set to the output waveform.
  *          This parameter can be one of the following values:
  *            @arg TIM_ForcedAction_Active: Force active level on OC5REF
  *            @arg TIM_ForcedAction_InActive: Force inactive level on OC5REF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ForcedOC6Config(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ForcedAction: uint16_t) {
    let mut tmpccmr3: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Reset the OC6M Bits */
    tmpccmr3 &= !(0x7000i32 as uint32_t);
    /* Configure The Forced output Mode */
    tmpccmr3 |= (TIM_ForcedAction as uint32_t) << 8i32;
    /* Write to TIMx CCMR3 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
}
/* *
  * @brief  Enables or disables the TIMx peripheral Preload register on CCR1.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPreload_Enable
  *            @arg TIM_OCPreload_Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC1PreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCPreload: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC1PE Bit */
    tmpccmr1 &= !(0x8i32 as uint32_t);
    /* Enable or Disable the Output Compare Preload feature */
    tmpccmr1 |= TIM_OCPreload as libc::c_uint;
    /* Write to TIMx CCMR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Enables or disables the TIMx peripheral Preload register on CCR2.
  * @param  TIMx: where x can be  1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPreload_Enable
  *            @arg TIM_OCPreload_Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC2PreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCPreload: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC2PE Bit */
    tmpccmr1 &= !(0x800i32 as uint32_t);
    /* Enable or Disable the Output Compare Preload feature */
    tmpccmr1 |= (TIM_OCPreload as uint32_t) << 8i32;
    /* Write to TIMx CCMR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Enables or disables the TIMx peripheral Preload register on CCR3.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPreload_Enable
  *            @arg TIM_OCPreload_Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC3PreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCPreload: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC3PE Bit */
    tmpccmr2 &= !(0x8i32 as uint32_t);
    /* Enable or Disable the Output Compare Preload feature */
    tmpccmr2 |= TIM_OCPreload as libc::c_uint;
    /* Write to TIMx CCMR2 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Enables or disables the TIMx peripheral Preload register on CCR4.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPreload_Enable
  *            @arg TIM_OCPreload_Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC4PreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCPreload: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC4PE Bit */
    tmpccmr2 &= !(0x800i32 as uint32_t);
    /* Enable or Disable the Output Compare Preload feature */
    tmpccmr2 |= (TIM_OCPreload as uint32_t) << 8i32;
    /* Write to TIMx CCMR2 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Enables or disables the TIMx peripheral Preload register on CCR5.
  * @param  TIMx: where x can be  1 or 8 to select the TIM peripheral.
  * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPreload_Enable
  *            @arg TIM_OCPreload_Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC5PreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCPreload: uint16_t) {
    let mut tmpccmr3: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Reset the OC5PE Bit */
    tmpccmr3 &= !(0x8i32 as uint32_t);
    /* Enable or Disable the Output Compare Preload feature */
    tmpccmr3 |= TIM_OCPreload as uint32_t;
    /* Write to TIMx CCMR3 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
}
/* *
  * @brief  Enables or disables the TIMx peripheral Preload register on CCR6.
  * @param  TIMx: where x can be  1 or 8 to select the TIM peripheral.
  * @param  TIM_OCPreload: new state of the TIMx peripheral Preload register
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPreload_Enable
  *            @arg TIM_OCPreload_Disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC6PreloadConfig(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCPreload: uint16_t) {
    let mut tmpccmr3: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Reset the OC5PE Bit */
    tmpccmr3 &= !(0x800i32 as uint32_t);
    /* Enable or Disable the Output Compare Preload feature */
    tmpccmr3 |= (TIM_OCPreload as uint32_t) << 8i32;
    /* Write to TIMx CCMR3 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
}
/* *
  * @brief  Configures the TIMx Output Compare 1 Fast feature.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCFast_Enable: TIM output compare fast enable
  *            @arg TIM_OCFast_Disable: TIM output compare fast disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC1FastConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_OCFast: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC1FE Bit */
    tmpccmr1 &= !(0x4i32 as uint32_t);
    /* Enable or Disable the Output Compare Fast Bit */
    tmpccmr1 |= TIM_OCFast as libc::c_uint;
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Configures the TIMx Output Compare 2 Fast feature.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCFast_Enable: TIM output compare fast enable
  *            @arg TIM_OCFast_Disable: TIM output compare fast disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC2FastConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_OCFast: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC2FE Bit */
    tmpccmr1 &= !(0x400i32 as uint32_t);
    /* Enable or Disable the Output Compare Fast Bit */
    tmpccmr1 |= (TIM_OCFast as uint32_t) << 8i32;
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Configures the TIMx Output Compare 3 Fast feature.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCFast_Enable: TIM output compare fast enable
  *            @arg TIM_OCFast_Disable: TIM output compare fast disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC3FastConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_OCFast: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the TIMx CCMR2 register value */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC3FE Bit */
    tmpccmr2 &= !(0x4i32 as uint32_t);
    /* Enable or Disable the Output Compare Fast Bit */
    tmpccmr2 |= TIM_OCFast as libc::c_uint;
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Configures the TIMx Output Compare 4 Fast feature.
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCFast: new state of the Output Compare Fast Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCFast_Enable: TIM output compare fast enable
  *            @arg TIM_OCFast_Disable: TIM output compare fast disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC4FastConfig(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_OCFast: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Get the TIMx CCMR2 register value */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC4FE Bit */
    tmpccmr2 &= !(0x400i32 as uint32_t);
    /* Enable or Disable the Output Compare Fast Bit */
    tmpccmr2 |= (TIM_OCFast as uint32_t) << 8i32;
    /* Write to TIMx CCMR2 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Clears or safeguards the OCREF1 signal on an external event
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCClear_Enable: TIM Output clear enable
  *            @arg TIM_OCClear_Disable: TIM Output clear disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearOC1Ref(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_OCClear: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC1CE Bit */
    tmpccmr1 &= !(0x80i32 as uint32_t);
    /* Enable or Disable the Output Compare Clear Bit */
    tmpccmr1 |= TIM_OCClear as libc::c_uint;
    /* Write to TIMx CCMR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Clears or safeguards the OCREF2 signal on an external event
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCClear_Enable: TIM Output clear enable
  *            @arg TIM_OCClear_Disable: TIM Output clear disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearOC2Ref(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_OCClear: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr1 = (*TIMx).CCMR1;
    /* Reset the OC2CE Bit */
    tmpccmr1 &= !(0x8000i32 as uint32_t);
    /* Enable or Disable the Output Compare Clear Bit */
    tmpccmr1 |= (TIM_OCClear as uint32_t) << 8i32;
    /* Write to TIMx CCMR1 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
}
/* *
  * @brief  Clears or safeguards the OCREF3 signal on an external event
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCClear_Enable: TIM Output clear enable
  *            @arg TIM_OCClear_Disable: TIM Output clear disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearOC3Ref(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_OCClear: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC3CE Bit */
    tmpccmr2 &= !(0x80i32 as uint32_t);
    /* Enable or Disable the Output Compare Clear Bit */
    tmpccmr2 |= TIM_OCClear as libc::c_uint;
    /* Write to TIMx CCMR2 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Clears or safeguards the OCREF4 signal on an external event
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCClear_Enable: TIM Output clear enable
  *            @arg TIM_OCClear_Disable: TIM Output clear disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearOC4Ref(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_OCClear: uint16_t) {
    let mut tmpccmr2: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr2 = (*TIMx).CCMR2;
    /* Reset the OC4CE Bit */
    tmpccmr2 &= !(0x8000i32 as uint32_t);
    /* Enable or Disable the Output Compare Clear Bit */
    tmpccmr2 |= (TIM_OCClear as uint32_t) << 8i32;
    /* Write to TIMx CCMR2 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2);
}
/* *
  * @brief  Clears or safeguards the OCREF5 signal on an external event
  * @param  TIMx: where x can be  1 or 8 to select the TIM peripheral.
  * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCClear_Enable: TIM Output clear enable
  *            @arg TIM_OCClear_Disable: TIM Output clear disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearOC5Ref(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_OCClear: uint16_t) {
    let mut tmpccmr3: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Reset the OC5CE Bit */
    tmpccmr3 &= !(0x80i32 as uint32_t);
    /* Enable or Disable the Output Compare Clear Bit */
    tmpccmr3 |= TIM_OCClear as uint32_t;
    /* Write to TIMx CCMR3 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
}
/* *
  * @brief  Clears or safeguards the OCREF6 signal on an external event
  * @param  TIMx: where x can be  1 or 8 to select the TIM peripheral.
  * @param  TIM_OCClear: new state of the Output Compare Clear Enable Bit.
  *          This parameter can be one of the following values:
  *            @arg TIM_OCClear_Enable: TIM Output clear enable
  *            @arg TIM_OCClear_Disable: TIM Output clear disable
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearOC6Ref(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_OCClear: uint16_t) {
    let mut tmpccmr3: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccmr3 = (*TIMx).CCMR3;
    /* Reset the OC5CE Bit */
    tmpccmr3 &= !(0x8000i32 as uint32_t);
    /* Enable or Disable the Output Compare Clear Bit */
    tmpccmr3 |= (TIM_OCClear as uint32_t) << 8i32;
    /* Write to TIMx CCMR3 register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR3 as *mut uint32_t,
                                tmpccmr3);
}
/* *
  * @brief  Selects the OCReference Clear source.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OCReferenceClear: specifies the OCReference Clear source.
  *   This parameter can be one of the following values:
  *     @arg TIM_OCReferenceClear_ETRF: The internal OCreference clear input is connected to ETRF.
  *     @arg TIM_OCReferenceClear_OCREFCLR: The internal OCreference clear input is connected to OCREF_CLR input.  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectOCREFClear(mut TIMx: *mut TIM_TypeDef,
                                              mut TIM_OCReferenceClear:
                                                  uint16_t) {
    /* Check the parameters */
    /* Set the TIM_OCReferenceClear source */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x8i32 as uint32_t as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TIM_OCReferenceClear as libc::c_uint) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the TIMx channel 1 polarity.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OCPolarity: specifies the OC1 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPolarity_High: Output Compare active high
  *            @arg TIM_OCPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC1PolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_OCPolarity: uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC1P Bit */
    tmpccer &= !(0x2i32 as uint32_t);
    tmpccer |= TIM_OCPolarity as libc::c_uint;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx Channel 1N polarity.
  * @param  TIMx: where x can be 1, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_OCNPolarity: specifies the OC1N Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCNPolarity_High: Output Compare active high
  *            @arg TIM_OCNPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC1NPolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                                mut TIM_OCNPolarity:
                                                    uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC1NP Bit */
    tmpccer &= !(0x8i32 as uint32_t);
    tmpccer |= TIM_OCNPolarity as libc::c_uint;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx channel 2 polarity.
  * @param  TIMx: where x can be 1, 2, 3, 4 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_OCPolarity: specifies the OC2 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPolarity_High: Output Compare active high
  *            @arg TIM_OCPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC2PolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_OCPolarity: uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC2P Bit */
    tmpccer &= !(0x20i32 as uint32_t);
    tmpccer |= (TIM_OCPolarity as uint32_t) << 4i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx Channel 2N polarity.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_OCNPolarity: specifies the OC2N Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCNPolarity_High: Output Compare active high
  *            @arg TIM_OCNPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC2NPolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                                mut TIM_OCNPolarity:
                                                    uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC2NP Bit */
    tmpccer &= !(0x80i32 as uint32_t);
    tmpccer |= (TIM_OCNPolarity as uint32_t) << 4i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx channel 3 polarity.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCPolarity: specifies the OC3 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPolarity_High: Output Compare active high
  *            @arg TIM_OCPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC3PolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_OCPolarity: uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC3P Bit */
    tmpccer &= !(0x200i32 as uint32_t);
    tmpccer |= (TIM_OCPolarity as uint32_t) << 8i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx Channel 3N polarity.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_OCNPolarity: specifies the OC3N Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCNPolarity_High: Output Compare active high
  *            @arg TIM_OCNPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC3NPolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                                mut TIM_OCNPolarity:
                                                    uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC3NP Bit */
    tmpccer &= !(0x800i32 as uint32_t);
    tmpccer |= (TIM_OCNPolarity as uint32_t) << 8i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx channel 4 polarity.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_OCPolarity: specifies the OC4 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPolarity_High: Output Compare active high
  *            @arg TIM_OCPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC4PolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_OCPolarity: uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC4P Bit */
    tmpccer &= !(0x2000i32 as uint32_t);
    tmpccer |= (TIM_OCPolarity as uint32_t) << 12i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx channel 5 polarity.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_OCPolarity: specifies the OC5 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPolarity_High: Output Compare active high
  *            @arg TIM_OCPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC5PolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_OCPolarity: uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC5P Bit */
    tmpccer &= !(0x20000i32 as uint32_t);
    tmpccer |= (TIM_OCPolarity as uint32_t) << 16i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configures the TIMx channel 6 polarity.
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  * @param  TIM_OCPolarity: specifies the OC6 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_OCPolarity_High: Output Compare active high
  *            @arg TIM_OCPolarity_Low: Output Compare active low
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_OC6PolarityConfig(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_OCPolarity: uint16_t) {
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmpccer = (*TIMx).CCER;
    /* Set or Reset the CC6P Bit */
    tmpccer &= !(0x200000i32 as uint32_t);
    tmpccer |= (TIM_OCPolarity as uint32_t) << 20i32;
    /* Write to TIMx CCER register */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Enables or disables the TIM Capture Compare Channel x.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  *            @arg TIM_Channel_4: TIM Channel 4
  *            @arg TIM_Channel_5: TIM Channel 5
  *            @arg TIM_Channel_6: TIM Channel 6    
  * @param  TIM_CCx: specifies the TIM Channel CCxE bit new state.
  *          This parameter can be: TIM_CCx_Enable or TIM_CCx_Disable. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_CCxCmd(mut TIMx: *mut TIM_TypeDef,
                                    mut TIM_Channel: uint16_t,
                                    mut TIM_CCx: uint16_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp = (0x1i32 as uint16_t as uint32_t) << TIM_Channel as uint32_t;
    /* Reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmp) as uint32_t as
                                    uint32_t);
    /* Set or reset the CCxE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (TIM_CCx as uint32_t) <<
                                         TIM_Channel as uint32_t) as uint32_t
                                    as uint32_t);
}
/* *
  * @brief  Enables or disables the TIM Capture Compare Channel xN.
  * @param  TIMx: where x can be 1, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_Channel: specifies the TIM Channel
  *          This parameter can be one of the following values:
  *            @arg TIM_Channel_1: TIM Channel 1
  *            @arg TIM_Channel_2: TIM Channel 2
  *            @arg TIM_Channel_3: TIM Channel 3
  * @param  TIM_CCxN: specifies the TIM Channel CCxNE bit new state.
  *          This parameter can be: TIM_CCxN_Enable or TIM_CCxN_Disable. 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_CCxNCmd(mut TIMx: *mut TIM_TypeDef,
                                     mut TIM_Channel: uint16_t,
                                     mut TIM_CCxN: uint16_t) {
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    tmp = (0x4i32 as uint16_t as uint32_t) << TIM_Channel as uint32_t;
    /* Reset the CCxNE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !tmp) as uint32_t as
                                    uint32_t);
    /* Set or reset the CCxNE Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (TIM_CCxN as uint32_t) <<
                                         TIM_Channel as uint32_t) as uint32_t
                                    as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup TIM_Group3 Input Capture management functions
 *  @brief    Input Capture management functions 
 *
@verbatim   
 ===============================================================================
               ##### Input Capture management functions #####
 ===============================================================================  
         
  *** TIM Driver: how to use it in Input Capture Mode ***
  =======================================================
  [..] 
  To use the Timer in Input Capture mode, the following steps are mandatory:
       
      (#) Enable TIM clock using RCC_APBxPeriphClockCmd(RCC_APBxPeriph_TIMx, ENABLE) function
       
      (#) Configure the TIM pins by configuring the corresponding GPIO pins
       
      (#) Configure the Time base unit as described in the first part of this driver,
          if needed, else the Timer will run with the default configuration:
          (++) Autoreload value = 0xFFFF
          (++) Prescaler value = 0x0000
          (++) Counter mode = Up counting
          (++) Clock Division = TIM_CKD_DIV1
          
      (#) Fill the TIM_ICInitStruct with the desired parameters including:
          (++) TIM Channel: TIM_Channel
          (++) TIM Input Capture polarity: TIM_ICPolarity
          (++) TIM Input Capture selection: TIM_ICSelection
          (++) TIM Input Capture Prescaler: TIM_ICPrescaler
          (++) TIM Input CApture filter value: TIM_ICFilter
       
      (#) Call TIM_ICInit(TIMx, &TIM_ICInitStruct) to configure the desired channel with the 
          corresponding configuration and to measure only frequency or duty cycle of the input signal,
          or,
          Call TIM_PWMIConfig(TIMx, &TIM_ICInitStruct) to configure the desired channels with the 
          corresponding configuration and to measure the frequency and the duty cycle of the input signal
          
      (#) Enable the NVIC or the DMA to read the measured frequency. 
          
      (#) Enable the corresponding interrupt (or DMA request) to read the Captured value,
          using the function TIM_ITConfig(TIMx, TIM_IT_CCx) (or TIM_DMA_Cmd(TIMx, TIM_DMA_CCx)) 
       
      (#) Call the TIM_Cmd(ENABLE) function to enable the TIM counter.
       
      (#) Use TIM_GetCapturex(TIMx); to read the captured value.
  [..]        
      (@) All other functions can be used separately to modify, if needed,
          a specific feature of the Timer. 

@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the TIM peripheral according to the specified parameters
  *         in the TIM_ICInitStruct.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ICInit(mut TIMx: *mut TIM_TypeDef,
                                    mut TIM_ICInitStruct:
                                        *mut TIM_ICInitTypeDef) {
    /* Check the parameters */
    if (*TIM_ICInitStruct).TIM_Channel as libc::c_int ==
           0i32 as uint16_t as libc::c_int {
        /* TI1 Configuration */
        TI1_Config(TIMx, (*TIM_ICInitStruct).TIM_ICPolarity,
                   (*TIM_ICInitStruct).TIM_ICSelection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC1Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
    } else if (*TIM_ICInitStruct).TIM_Channel as libc::c_int ==
                  0x4i32 as uint16_t as libc::c_int {
        /* TI2 Configuration */
        TI2_Config(TIMx, (*TIM_ICInitStruct).TIM_ICPolarity,
                   (*TIM_ICInitStruct).TIM_ICSelection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC2Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
    } else if (*TIM_ICInitStruct).TIM_Channel as libc::c_int ==
                  0x8i32 as uint16_t as libc::c_int {
        /* TI3 Configuration */
        TI3_Config(TIMx, (*TIM_ICInitStruct).TIM_ICPolarity,
                   (*TIM_ICInitStruct).TIM_ICSelection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC3Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
    } else {
        /* TI4 Configuration */
        TI4_Config(TIMx, (*TIM_ICInitStruct).TIM_ICPolarity,
                   (*TIM_ICInitStruct).TIM_ICSelection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC4Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
    };
}
/* *
  * @brief  Fills each TIM_ICInitStruct member with its default value.
  * @param  TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ICStructInit(mut TIM_ICInitStruct:
                                              *mut TIM_ICInitTypeDef) {
    /* Set the default configuration */
    (*TIM_ICInitStruct).TIM_Channel = 0i32 as uint16_t;
    (*TIM_ICInitStruct).TIM_ICPolarity = 0i32 as uint16_t;
    (*TIM_ICInitStruct).TIM_ICSelection = 0x1i32 as uint16_t;
    (*TIM_ICInitStruct).TIM_ICPrescaler = 0i32 as uint16_t;
    (*TIM_ICInitStruct).TIM_ICFilter = 0i32 as uint16_t;
}
/* *
  * @brief  Configures the TIM peripheral according to the specified parameters
  *         in the TIM_ICInitStruct to measure an external PWM signal.
  * @param  TIMx: where x can be  1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_ICInitStruct: pointer to a TIM_ICInitTypeDef structure that contains
  *         the configuration information for the specified TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_PWMIConfig(mut TIMx: *mut TIM_TypeDef,
                                        mut TIM_ICInitStruct:
                                            *mut TIM_ICInitTypeDef) {
    let mut icoppositepolarity: uint16_t = 0i32 as uint16_t;
    let mut icoppositeselection: uint16_t = 0x1i32 as uint16_t;
    /* Check the parameters */
    /* Select the Opposite Input Polarity */
    if (*TIM_ICInitStruct).TIM_ICPolarity as libc::c_int ==
           0i32 as uint16_t as libc::c_int {
        icoppositepolarity = 0x2i32 as uint16_t
    } else { icoppositepolarity = 0i32 as uint16_t }
    /* Select the Opposite Input */
    if (*TIM_ICInitStruct).TIM_ICSelection as libc::c_int ==
           0x1i32 as uint16_t as libc::c_int {
        icoppositeselection = 0x2i32 as uint16_t
    } else { icoppositeselection = 0x1i32 as uint16_t }
    if (*TIM_ICInitStruct).TIM_Channel as libc::c_int ==
           0i32 as uint16_t as libc::c_int {
        /* TI1 Configuration */
        TI1_Config(TIMx, (*TIM_ICInitStruct).TIM_ICPolarity,
                   (*TIM_ICInitStruct).TIM_ICSelection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC1Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
        /* TI2 Configuration */
        TI2_Config(TIMx, icoppositepolarity, icoppositeselection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC2Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
    } else {
        /* TI2 Configuration */
        TI2_Config(TIMx, (*TIM_ICInitStruct).TIM_ICPolarity,
                   (*TIM_ICInitStruct).TIM_ICSelection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC2Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
        /* TI1 Configuration */
        TI1_Config(TIMx, icoppositepolarity, icoppositeselection,
                   (*TIM_ICInitStruct).TIM_ICFilter);
        /* Set the Input Capture Prescaler value */
        TIM_SetIC1Prescaler(TIMx, (*TIM_ICInitStruct).TIM_ICPrescaler);
    };
}
/* *
  * @brief  Gets the TIMx Input Capture 1 value.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @retval Capture Compare 1 Register value.
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetCapture1(mut TIMx: *mut TIM_TypeDef)
 -> uint32_t {
    /* Check the parameters */
    /* Get the Capture 1 Register value */
    return (*TIMx).CCR1;
}
/* *
  * @brief  Gets the TIMx Input Capture 2 value.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @retval Capture Compare 2 Register value.
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetCapture2(mut TIMx: *mut TIM_TypeDef)
 -> uint32_t {
    /* Check the parameters */
    /* Get the Capture 2 Register value */
    return (*TIMx).CCR2;
}
/* *
  * @brief  Gets the TIMx Input Capture 3 value.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @retval Capture Compare 3 Register value.
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetCapture3(mut TIMx: *mut TIM_TypeDef)
 -> uint32_t {
    /* Check the parameters */
    /* Get the Capture 3 Register value */
    return (*TIMx).CCR3;
}
/* *
  * @brief  Gets the TIMx Input Capture 4 value.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @retval Capture Compare 4 Register value.
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetCapture4(mut TIMx: *mut TIM_TypeDef)
 -> uint32_t {
    /* Check the parameters */
    /* Get the Capture 4 Register value */
    return (*TIMx).CCR4;
}
/* *
  * @brief  Sets the TIMx Input Capture 1 prescaler.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_ICPSC: specifies the Input Capture1 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPSC_DIV1: no prescaler
  *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetIC1Prescaler(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ICPSC: uint16_t) {
    /* Check the parameters */
    /* Reset the IC1PSC Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0xci32 as uint32_t))
                                    as uint32_t as uint32_t);
    /* Set the IC1PSC value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TIM_ICPSC as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Sets the TIMx Input Capture 2 prescaler.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_ICPSC: specifies the Input Capture2 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPSC_DIV1: no prescaler
  *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetIC2Prescaler(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ICPSC: uint16_t) {
    /* Check the parameters */
    /* Reset the IC2PSC Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xc00i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Set the IC2PSC value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR1
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (TIM_ICPSC as uint32_t) << 8i32) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Sets the TIMx Input Capture 3 prescaler.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ICPSC: specifies the Input Capture3 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPSC_DIV1: no prescaler
  *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetIC3Prescaler(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ICPSC: uint16_t) {
    /* Check the parameters */
    /* Reset the IC3PSC Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xci32 as uint16_t as libc::c_int) as
                                         uint16_t as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Set the IC3PSC value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TIM_ICPSC as libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Sets the TIMx Input Capture 4 prescaler.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ICPSC: specifies the Input Capture4 prescaler new value.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPSC_DIV1: no prescaler
  *            @arg TIM_ICPSC_DIV2: capture is done once every 2 events
  *            @arg TIM_ICPSC_DIV4: capture is done once every 4 events
  *            @arg TIM_ICPSC_DIV8: capture is done once every 8 events
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SetIC4Prescaler(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_ICPSC: uint16_t) {
    /* Check the parameters */
    /* Reset the IC4PSC Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xc00i32 as uint16_t as libc::c_int) as
                                         uint16_t as libc::c_uint) as uint32_t
                                    as uint32_t);
    /* Set the IC4PSC value */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCMR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     ((TIM_ICPSC as libc::c_int) << 8i32) as
                                         uint16_t as libc::c_uint) as uint32_t
                                    as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup TIM_Group4 Advanced-control timers (TIM1 and TIM8) specific features
 *  @brief   Advanced-control timers (TIM1 and TIM8) specific features
 *
@verbatim   
 ===============================================================================
     ##### Advanced-control timers (TIM1 and TIM8) specific features #####
 ===============================================================================  
       
  *** TIM Driver: how to use the Break feature ***
  ================================================ 
  [..]
  After configuring the Timer channel(s) in the appropriate Output Compare mode: 
                         
       (#) Fill the TIM_BDTRInitStruct with the desired parameters for the Timer
           Break Polarity, dead time, Lock level, the OSSI/OSSR State and the 
           AOE(automatic output enable).
               
       (#) Call TIM_BDTRConfig(TIMx, &TIM_BDTRInitStruct) to configure the Timer
          
       (#) Enable the Main Output using TIM_CtrlPWMOutputs(TIM1, ENABLE) 
          
       (#) Once the break even occurs, the Timer's output signals are put in reset
           state or in a known state (according to the configuration made in
           TIM_BDTRConfig() function).

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the Break feature, dead time, Lock level, OSSI/OSSR State
  *         and the AOE(automatic output enable).
  * @param  TIMx: where x can be  1, 8, 15, 16 or 17 to select the TIM 
  * @param  TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef structure that
  *         contains the BDTR Register configuration  information for the TIM peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_BDTRConfig(mut TIMx: *mut TIM_TypeDef,
                                        mut TIM_BDTRInitStruct:
                                            *mut TIM_BDTRInitTypeDef) {
    /* Check the parameters */
    /* Set the Lock level, the Break enable Bit and the Polarity, the OSSR State,
     the OSSI State, the dead time value and the Automatic Output Enable Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                (*TIM_BDTRInitStruct).TIM_OSSRState as
                                    uint32_t |
                                    (*TIM_BDTRInitStruct).TIM_OSSIState as
                                        libc::c_uint |
                                    (*TIM_BDTRInitStruct).TIM_LOCKLevel as
                                        libc::c_uint |
                                    (*TIM_BDTRInitStruct).TIM_DeadTime as
                                        libc::c_uint |
                                    (*TIM_BDTRInitStruct).TIM_Break as
                                        libc::c_uint |
                                    (*TIM_BDTRInitStruct).TIM_BreakPolarity as
                                        libc::c_uint |
                                    (*TIM_BDTRInitStruct).TIM_AutomaticOutput
                                        as libc::c_uint);
}
/* *
  * @brief  Configures the Break1 feature.
  * @param  TIMx: where x can be  1 or 8 to select the TIM 
  * @param  TIM_Break1Polarity: specifies the Break1 polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_Break1Polarity_Low: Break1 input is active low
  *            @arg TIM_Break1Polarity_High: Break1 input is active high
  * @param  TIM_Break1Filter: specifies the Break1 filter value.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_Break1Config(mut TIMx: *mut TIM_TypeDef,
                                          mut TIM_Break1Polarity: uint32_t,
                                          mut TIM_Break1Filter: uint8_t) {
    /* Check the parameters */
    /* Reset the BKP and BKF Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x2000i32 as uint32_t |
                                           0xf0000i32 as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Configure the Break1 polarity and filter */
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (TIM_Break1Polarity |
                                          (TIM_Break1Filter as uint32_t) <<
                                              16i32)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Configures the Break2 feature.
  * @param  TIMx: where x can be  1 or 8 to select the TIM 
  * @param  TIM_Break2Polarity: specifies the Break2 polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_Break2Polarity_Low: Break2 input is active low
  *            @arg TIM_Break2Polarity_High: Break2 input is active high
  * @param  TIM_Break2Filter: specifies the Break2 filter value.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_Break2Config(mut TIMx: *mut TIM_TypeDef,
                                          mut TIM_Break2Polarity: uint32_t,
                                          mut TIM_Break2Filter: uint8_t) {
    /* Check the parameters */
    /* Reset the BKP and BKF Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x2000000i32 as uint32_t |
                                           0xf00000i32 as uint32_t)) as
                                    uint32_t as uint32_t);
    /* Configure the Break1 polarity and filter */
    ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (TIM_Break2Polarity |
                                          (TIM_Break2Filter as uint32_t) <<
                                              20i32)) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Enables or disables the TIM Break1 input.
  * @param  TIMx: where x can be 1, 8, 1, 16 or 17 to select the TIMx peripheral.
  * @param  NewState: new state of the TIM Break1 input.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_Break1Cmd(mut TIMx: *mut TIM_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Break1 */
        ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the Break1 */
        ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000i32 as uint32_t)) as uint32_t
                                        as uint32_t)
    };
}
/* *
  * @brief  Enables or disables the TIM Break2 input.
  * @param  TIMx: where x can be 1 or 8 to select the TIMx peripheral.
  * @param  NewState: new state of the TIM Break2 input.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_Break2Cmd(mut TIMx: *mut TIM_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Break1 */
        ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1000000i32 as uint32_t) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the Break1 */
        ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1000000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Fills each TIM_BDTRInitStruct member with its default value.
  * @param  TIM_BDTRInitStruct: pointer to a TIM_BDTRInitTypeDef structure which
  *         will be initialized.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_BDTRStructInit(mut TIM_BDTRInitStruct:
                                                *mut TIM_BDTRInitTypeDef) {
    /* Set the default configuration */
    (*TIM_BDTRInitStruct).TIM_OSSRState = 0i32 as uint16_t;
    (*TIM_BDTRInitStruct).TIM_OSSIState = 0i32 as uint16_t;
    (*TIM_BDTRInitStruct).TIM_LOCKLevel = 0i32 as uint16_t;
    (*TIM_BDTRInitStruct).TIM_DeadTime = 0i32 as uint16_t;
    (*TIM_BDTRInitStruct).TIM_Break = 0i32 as uint16_t;
    (*TIM_BDTRInitStruct).TIM_BreakPolarity = 0i32 as uint16_t;
    (*TIM_BDTRInitStruct).TIM_AutomaticOutput = 0i32 as uint16_t;
}
/* *
  * @brief  Enables or disables the TIM peripheral Main Outputs.
  * @param  TIMx: where x can be 1, 8, 15, 16 or 17 to select the TIMx peripheral.
  * @param  NewState: new state of the TIM peripheral Main Outputs.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_CtrlPWMOutputs(mut TIMx: *mut TIM_TypeDef,
                                            mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the TIM Main Output */
        ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x8000i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Disable the TIM Main Output */
        ::core::ptr::write_volatile(&mut (*TIMx).BDTR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).BDTR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8000i32 as uint32_t) as uint16_t
                                             as libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Selects the TIM peripheral Commutation event.
  * @param  TIMx: where x can be  1, 8, 15, 16 or 17 to select the TIMx peripheral
  * @param  NewState: new state of the Commutation event.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectCOM(mut TIMx: *mut TIM_TypeDef,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the COM Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x4i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Reset the COM Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x4i32 as uint32_t) as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Sets or Resets the TIM peripheral Capture Compare Preload Control bit.
  * @param  TIMx: where x can be  1 or 8 to select the TIMx peripheral
  * @param  NewState: new state of the Capture Compare Preload Control bit
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_CCPreloadControl(mut TIMx: *mut TIM_TypeDef,
                                              mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the CCPC Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x1i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Reset the CCPC Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x1i32 as uint32_t) as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup TIM_Group5 Interrupts DMA and flags management functions
 *  @brief    Interrupts, DMA and flags management functions 
 *
@verbatim   
 ===============================================================================
         ##### Interrupts, DMA and flags management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables or disables the specified TIM interrupts.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIMx peripheral.
  * @param  TIM_IT: specifies the TIM interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg TIM_IT_Update: TIM update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *  
  * @note   For TIM6 and TIM7 only the parameter TIM_IT_Update can be used
  * @note   For TIM9 and TIM12 only one of the following parameters can be used: TIM_IT_Update,
  *          TIM_IT_CC1, TIM_IT_CC2 or TIM_IT_Trigger. 
  * @note   For TIM10, TIM11, TIM13 and TIM14 only one of the following parameters can
  *          be used: TIM_IT_Update or TIM_IT_CC1   
  * @note   TIM_IT_COM and TIM_IT_Break can be used only with TIM1 and TIM8 
  *        
  * @param  NewState: new state of the TIM interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ITConfig(mut TIMx: *mut TIM_TypeDef,
                                      mut TIM_IT: uint16_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the Interrupt sources */
        ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         TIM_IT as libc::c_uint) as uint32_t
                                        as uint32_t)
    } else {
        /* Disable the Interrupt sources */
        ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(TIM_IT as libc::c_int) as uint16_t
                                             as libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @brief  Configures the TIMx event to be generate by software.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_EventSource: specifies the event source.
  *          This parameter can be one or more of the following values:	   
  *            @arg TIM_EventSource_Update: Timer update Event source
  *            @arg TIM_EventSource_CC1: Timer Capture Compare 1 Event source
  *            @arg TIM_EventSource_CC2: Timer Capture Compare 2 Event source
  *            @arg TIM_EventSource_CC3: Timer Capture Compare 3 Event source
  *            @arg TIM_EventSource_CC4: Timer Capture Compare 4 Event source
  *            @arg TIM_EventSource_COM: Timer COM event source  
  *            @arg TIM_EventSource_Trigger: Timer Trigger Event source
  *            @arg TIM_EventSource_Break: Timer Break event source
  * 
  * @note   TIM6 and TIM7 can only generate an update event. 
  * @note   TIM_EventSource_COM and TIM_EventSource_Break are used only with TIM1 and TIM8.
  *        
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GenerateEvent(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_EventSource: uint16_t) {
    /* Check the parameters */
    /* Set the event sources */
    ::core::ptr::write_volatile(&mut (*TIMx).EGR as *mut uint32_t,
                                TIM_EventSource as uint32_t);
}
/* *
  * @brief  Checks whether the specified TIM flag is set or not.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_FLAG: specifies the flag to check.
  *          This parameter can be one of the following values:
  *            @arg TIM_FLAG_Update: TIM update Flag
  *            @arg TIM_FLAG_CC1: TIM Capture Compare 1 Flag
  *            @arg TIM_FLAG_CC2: TIM Capture Compare 2 Flag
  *            @arg TIM_FLAG_CC3: TIM Capture Compare 3 Flag
  *            @arg TIM_FLAG_CC4: TIM Capture Compare 4 Flag
  *            @arg TIM_FLAG_CC5: TIM Capture Compare 5 Flag
  *            @arg TIM_FLAG_CC6: TIM Capture Compare 6 Flag  
  *            @arg TIM_FLAG_COM: TIM Commutation Flag
  *            @arg TIM_FLAG_Trigger: TIM Trigger Flag
  *            @arg TIM_FLAG_Break: TIM Break Flag
  *            @arg TIM_FLAG_CC1OF: TIM Capture Compare 1 over capture Flag
  *            @arg TIM_FLAG_CC2OF: TIM Capture Compare 2 over capture Flag
  *            @arg TIM_FLAG_CC3OF: TIM Capture Compare 3 over capture Flag
  *            @arg TIM_FLAG_CC4OF: TIM Capture Compare 4 over capture Flag
  *
  * @note   TIM6 and TIM7 can have only one update flag. 
  * @note   TIM_FLAG_COM and TIM_FLAG_Break are used only with TIM1 and TIM8.    
  *
  * @retval The new state of TIM_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetFlagStatus(mut TIMx: *mut TIM_TypeDef,
                                           mut TIM_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    if (*TIMx).SR & TIM_FLAG != RESET as libc::c_int as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the TIMx's pending flags.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_FLAG: specifies the flag bit to clear.
  *          This parameter can be any combination of the following values:
  *            @arg TIM_FLAG_Update: TIM update Flag
  *            @arg TIM_FLAG_CC1: TIM Capture Compare 1 Flag
  *            @arg TIM_FLAG_CC2: TIM Capture Compare 2 Flag
  *            @arg TIM_FLAG_CC3: TIM Capture Compare 3 Flag
  *            @arg TIM_FLAG_CC4: TIM Capture Compare 4 Flag
  *            @arg TIM_FLAG_CC5: TIM Capture Compare 5 Flag
  *            @arg TIM_FLAG_CC6: TIM Capture Compare 6 Flag               
  *            @arg TIM_FLAG_COM: TIM Commutation Flag
  *            @arg TIM_FLAG_Trigger: TIM Trigger Flag
  *            @arg TIM_FLAG_Break: TIM Break Flag
  *            @arg TIM_FLAG_CC1OF: TIM Capture Compare 1 over capture Flag
  *            @arg TIM_FLAG_CC2OF: TIM Capture Compare 2 over capture Flag
  *            @arg TIM_FLAG_CC3OF: TIM Capture Compare 3 over capture Flag
  *            @arg TIM_FLAG_CC4OF: TIM Capture Compare 4 over capture Flag
  *
  * @note   TIM6 and TIM7 can have only one update flag. 
  * @note   TIM_FLAG_COM and TIM_FLAG_Break are used only with TIM1 and TIM8.
  *    
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearFlag(mut TIMx: *mut TIM_TypeDef,
                                       mut TIM_FLAG: uint16_t) {
    /* Check the parameters */
    /* Clear the flags */
    ::core::ptr::write_volatile(&mut (*TIMx).SR as *mut uint32_t,
                                !(TIM_FLAG as libc::c_int) as uint16_t as
                                    uint32_t);
}
/* *
  * @brief  Checks whether the TIM interrupt has occurred or not.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_IT: specifies the TIM interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg TIM_IT_Update: TIM update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *
  * @note   TIM6 and TIM7 can generate only an update interrupt.
  * @note   TIM_IT_COM and TIM_IT_Break are used only with TIM1 and TIM8.
  *     
  * @retval The new state of the TIM_IT(SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_GetITStatus(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_IT: uint16_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itstatus: uint16_t = 0i32 as uint16_t;
    let mut itenable: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    itstatus = ((*TIMx).SR & TIM_IT as libc::c_uint) as uint16_t;
    itenable = ((*TIMx).DIER & TIM_IT as libc::c_uint) as uint16_t;
    if itstatus as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int &&
           itenable as libc::c_int !=
               RESET as libc::c_int as uint16_t as libc::c_int {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the TIMx's interrupt pending bits.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_IT: specifies the pending bit to clear.
  *          This parameter can be any combination of the following values:
  *            @arg TIM_IT_Update: TIM1 update Interrupt source
  *            @arg TIM_IT_CC1: TIM Capture Compare 1 Interrupt source
  *            @arg TIM_IT_CC2: TIM Capture Compare 2 Interrupt source
  *            @arg TIM_IT_CC3: TIM Capture Compare 3 Interrupt source
  *            @arg TIM_IT_CC4: TIM Capture Compare 4 Interrupt source
  *            @arg TIM_IT_COM: TIM Commutation Interrupt source
  *            @arg TIM_IT_Trigger: TIM Trigger Interrupt source
  *            @arg TIM_IT_Break: TIM Break Interrupt source
  *
  * @note   TIM6 and TIM7 can generate only an update interrupt.
  * @note   TIM_IT_COM and TIM_IT_Break are used only with TIM1 and TIM8.
  *      
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ClearITPendingBit(mut TIMx: *mut TIM_TypeDef,
                                               mut TIM_IT: uint16_t) {
    /* Check the parameters */
    /* Clear the IT pending Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).SR as *mut uint32_t,
                                !(TIM_IT as libc::c_int) as uint16_t as
                                    uint32_t);
}
/* *
  * @brief  Configures the TIMx's DMA interface.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_DMABase: DMA Base address.
  *          This parameter can be one of the following values:
  *            @arg TIM_DMABase_CR1  
  *            @arg TIM_DMABase_CR2
  *            @arg TIM_DMABase_SMCR
  *            @arg TIM_DMABase_DIER
  *            @arg TIM1_DMABase_SR
  *            @arg TIM_DMABase_EGR
  *            @arg TIM_DMABase_CCMR1
  *            @arg TIM_DMABase_CCMR2
  *            @arg TIM_DMABase_CCER
  *            @arg TIM_DMABase_CNT   
  *            @arg TIM_DMABase_PSC   
  *            @arg TIM_DMABase_ARR
  *            @arg TIM_DMABase_RCR
  *            @arg TIM_DMABase_CCR1
  *            @arg TIM_DMABase_CCR2
  *            @arg TIM_DMABase_CCR3  
  *            @arg TIM_DMABase_CCR4
  *            @arg TIM_DMABase_BDTR
  *            @arg TIM_DMABase_DCR
  * @param  TIM_DMABurstLength: DMA Burst length. This parameter can be one value
  *         between: TIM_DMABurstLength_1Transfer and TIM_DMABurstLength_18Transfers.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_DMAConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut TIM_DMABase: uint16_t,
                                       mut TIM_DMABurstLength: uint16_t) {
    /* Check the parameters */
    /* Set the DMA Base and the DMA Burst Length */
    ::core::ptr::write_volatile(&mut (*TIMx).DCR as *mut uint16_t,
                                (TIM_DMABase as libc::c_int |
                                     TIM_DMABurstLength as libc::c_int) as
                                    uint16_t);
}
/* *
  * @brief  Enables or disables the TIMx's DMA Requests.
  * @param  TIMx: where x can be 1, 2, 3, 4, 6, 7, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  TIM_DMASource: specifies the DMA Request sources.
  *          This parameter can be any combination of the following values:
  *            @arg TIM_DMA_Update: TIM update Interrupt source
  *            @arg TIM_DMA_CC1: TIM Capture Compare 1 DMA source
  *            @arg TIM_DMA_CC2: TIM Capture Compare 2 DMA source
  *            @arg TIM_DMA_CC3: TIM Capture Compare 3 DMA source
  *            @arg TIM_DMA_CC4: TIM Capture Compare 4 DMA source
  *            @arg TIM_DMA_COM: TIM Commutation DMA source
  *            @arg TIM_DMA_Trigger: TIM Trigger DMA source
  * @param  NewState: new state of the DMA Request sources.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_DMACmd(mut TIMx: *mut TIM_TypeDef,
                                    mut TIM_DMASource: uint16_t,
                                    mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Enable the DMA sources */
        ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         TIM_DMASource as libc::c_uint) as
                                        uint32_t as uint32_t)
    } else {
        /* Disable the DMA sources */
        ::core::ptr::write_volatile(&mut (*TIMx).DIER as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).DIER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(TIM_DMASource as libc::c_int) as
                                             uint16_t as libc::c_uint) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Selects the TIMx peripheral Capture Compare DMA source.
  * @param  TIMx: where x can be  1, 2, 3, 4, 8, 15, 16 or 17 to select the TIM peripheral.
  * @param  NewState: new state of the Capture Compare DMA source
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectCCDMA(mut TIMx: *mut TIM_TypeDef,
                                         mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the CCDS Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | 0x8i32 as uint32_t)
                                        as uint32_t as uint32_t)
    } else {
        /* Reset the CCDS Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x8i32 as uint32_t) as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  * @}
  */
/* * @defgroup TIM_Group6 Clocks management functions
 *  @brief    Clocks management functions
 *
@verbatim   
 ===============================================================================
                   ##### Clocks management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the TIMx internal Clock
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_InternalClockConfig(mut TIMx: *mut TIM_TypeDef) {
    /* Check the parameters */
    /* Disable slave mode to clock the prescaler directly with the internal clock */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x10007i32 as uint32_t) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
}
/* *
  * @brief  Configures the TIMx Internal Trigger as External Clock
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  TIM_InputTriggerSource: Trigger source.
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal Trigger 0
  *            @arg TIM_TS_ITR1: Internal Trigger 1
  *            @arg TIM_TS_ITR2: Internal Trigger 2
  *            @arg TIM_TS_ITR3: Internal Trigger 3
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ITRxExternalClockConfig(mut TIMx:
                                                         *mut TIM_TypeDef,
                                                     mut TIM_InputTriggerSource:
                                                         uint16_t) {
    /* Check the parameters */
    /* Select the Internal Trigger */
    TIM_SelectInputTrigger(TIMx, TIM_InputTriggerSource);
    /* Select the External clock mode1 */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x7i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the TIMx Trigger as External Clock
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15  
  *         to select the TIM peripheral.
  * @param  TIM_TIxExternalCLKSource: Trigger source.
  *          This parameter can be one of the following values:
  *            @arg TIM_TIxExternalCLK1Source_TI1ED: TI1 Edge Detector
  *            @arg TIM_TIxExternalCLK1Source_TI1: Filtered Timer Input 1
  *            @arg TIM_TIxExternalCLK1Source_TI2: Filtered Timer Input 2
  * @param  TIM_ICPolarity: specifies the TIx Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  * @param  ICFilter: specifies the filter value.
  *          This parameter must be a value between 0x0 and 0xF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_TIxExternalClockConfig(mut TIMx:
                                                        *mut TIM_TypeDef,
                                                    mut TIM_TIxExternalCLKSource:
                                                        uint16_t,
                                                    mut TIM_ICPolarity:
                                                        uint16_t,
                                                    mut ICFilter: uint16_t) {
    /* Check the parameters */
    /* Configure the Timer Input Clock Source */
    if TIM_TIxExternalCLKSource as libc::c_int ==
           0x60i32 as uint16_t as libc::c_int {
        TI2_Config(TIMx, TIM_ICPolarity, 0x1i32 as uint16_t, ICFilter);
    } else { TI1_Config(TIMx, TIM_ICPolarity, 0x1i32 as uint16_t, ICFilter); }
    /* Select the Trigger source */
    TIM_SelectInputTrigger(TIMx, TIM_TIxExternalCLKSource);
    /* Select the External clock mode1 */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x7i32 as uint32_t) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the External clock Mode1
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
  *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
  *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
  *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
  * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
  *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
  * @param  ExtTRGFilter: External Trigger Filter.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ETRClockMode1Config(mut TIMx: *mut TIM_TypeDef,
                                                 mut TIM_ExtTRGPrescaler:
                                                     uint16_t,
                                                 mut TIM_ExtTRGPolarity:
                                                     uint16_t,
                                                 mut ExtTRGFilter: uint16_t) {
    let mut tmpsmcr: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    /* Configure the ETR Clock source */
    TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity,
                  ExtTRGFilter);
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*TIMx).SMCR as uint16_t;
    /* Reset the SMS Bits */
    tmpsmcr =
        (tmpsmcr as libc::c_int &
             !(0x10007i32 as uint32_t) as uint16_t as libc::c_int) as
            uint16_t;
    /* Select the External clock mode1 */
    tmpsmcr = (tmpsmcr as libc::c_uint | 0x7i32 as uint32_t) as uint16_t;
    /* Select the Trigger selection : ETRF */
    tmpsmcr =
        (tmpsmcr as libc::c_int &
             !(0x70i32 as uint32_t) as uint16_t as libc::c_int) as uint16_t;
    tmpsmcr =
        (tmpsmcr as libc::c_int | 0x70i32 as uint16_t as libc::c_int) as
            uint16_t;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                tmpsmcr as uint32_t);
}
/* *
  * @brief  Configures the External clock Mode2
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
  *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
  *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
  *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
  * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
  *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
  * @param  ExtTRGFilter: External Trigger Filter.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ETRClockMode2Config(mut TIMx: *mut TIM_TypeDef,
                                                 mut TIM_ExtTRGPrescaler:
                                                     uint16_t,
                                                 mut TIM_ExtTRGPolarity:
                                                     uint16_t,
                                                 mut ExtTRGFilter: uint16_t) {
    /* Check the parameters */
    /* Configure the ETR Clock source */
    TIM_ETRConfig(TIMx, TIM_ExtTRGPrescaler, TIM_ExtTRGPolarity,
                  ExtTRGFilter);
    /* Enable the External clock mode2 */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | 0x4000i32 as uint32_t)
                                    as uint32_t as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup TIM_Group7 Synchronization management functions
 *  @brief    Synchronization management functions 
 *
@verbatim   
 ===============================================================================
                 ##### Synchronization management functions #####
 ===============================================================================  
                         
  *** TIM Driver: how to use it in synchronization Mode ***
  ========================================================= 
  [..] Case of two/several Timers
       
       (#) Configure the Master Timers using the following functions:
           (++) void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource); 
           (++) void TIM_SelectMasterSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_MasterSlaveMode);  
       (#) Configure the Slave Timers using the following functions: 
           (++) void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);  
           (++) void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode); 
          
  [..] Case of Timers and external trigger(ETR pin)
           
       (#) Configure the External trigger using this function:
           (++) void TIM_ETRConfig(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler, uint16_t TIM_ExtTRGPolarity,
                                   uint16_t ExtTRGFilter);
       (#) Configure the Slave Timers using the following functions: 
           (++) void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);  
           (++) void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode); 

@endverbatim
  * @{
  */
/* *
  * @brief  Selects the Input Trigger source
  * @param  TIMx: where x can be  1, 2, 3, 4, 8 or 15  
  *         to select the TIM peripheral.
  * @param  TIM_InputTriggerSource: The Input Trigger source.
  *          This parameter can be one of the following values:
  *            @arg TIM_TS_ITR0: Internal Trigger 0
  *            @arg TIM_TS_ITR1: Internal Trigger 1
  *            @arg TIM_TS_ITR2: Internal Trigger 2
  *            @arg TIM_TS_ITR3: Internal Trigger 3
  *            @arg TIM_TS_TI1F_ED: TI1 Edge Detector
  *            @arg TIM_TS_TI1FP1: Filtered Timer Input 1
  *            @arg TIM_TS_TI2FP2: Filtered Timer Input 2
  *            @arg TIM_TS_ETRF: External Trigger input
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectInputTrigger(mut TIMx: *mut TIM_TypeDef,
                                                mut TIM_InputTriggerSource:
                                                    uint16_t) {
    let mut tmpsmcr: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*TIMx).SMCR as uint16_t;
    /* Reset the TS Bits */
    tmpsmcr =
        (tmpsmcr as libc::c_int &
             !(0x70i32 as uint32_t) as uint16_t as libc::c_int) as uint16_t;
    /* Set the Input Trigger source */
    tmpsmcr =
        (tmpsmcr as libc::c_int | TIM_InputTriggerSource as libc::c_int) as
            uint16_t;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                tmpsmcr as uint32_t);
}
/* *
  * @brief  Selects the TIMx Trigger Output Mode.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5, 6, 7, 8 or 15 to select the TIM peripheral.
  *     
  * @param  TIM_TRGOSource: specifies the Trigger Output source.
  *   This parameter can be one of the following values:
  *
  *  - For all TIMx
  *            @arg TIM_TRGOSource_Reset:  The UG bit in the TIM_EGR register is used as the trigger output(TRGO)
  *            @arg TIM_TRGOSource_Enable: The Counter Enable CEN is used as the trigger output(TRGO)
  *            @arg TIM_TRGOSource_Update: The update event is selected as the trigger output(TRGO)
  *
  *  - For all TIMx except TIM6 and TIM7
  *            @arg TIM_TRGOSource_OC1: The trigger output sends a positive pulse when the CC1IF flag
  *                                     is to be set, as soon as a capture or compare match occurs(TRGO)
  *            @arg TIM_TRGOSource_OC1Ref: OC1REF signal is used as the trigger output(TRGO)
  *            @arg TIM_TRGOSource_OC2Ref: OC2REF signal is used as the trigger output(TRGO)
  *            @arg TIM_TRGOSource_OC3Ref: OC3REF signal is used as the trigger output(TRGO)
  *            @arg TIM_TRGOSource_OC4Ref: OC4REF signal is used as the trigger output(TRGO)
  *
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectOutputTrigger(mut TIMx: *mut TIM_TypeDef,
                                                 mut TIM_TRGOSource:
                                                     uint16_t) {
    /* Check the parameters */
    /* Reset the MMS Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x70i32 as uint32_t) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Select the TRGO source */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TIM_TRGOSource as libc::c_uint) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Selects the TIMx Trigger Output Mode2 (TRGO2).
  * @param  TIMx: where x can be 1 or 8 to select the TIM peripheral.
  *     
  * @param  TIM_TRGO2Source: specifies the Trigger Output source.
  *   This parameter can be one of the following values:
  *
  *  - For all TIMx
  *            @arg TIM_TRGOSource_Reset:  The UG bit in the TIM_EGR register is used as the trigger output(TRGO2)
  *            @arg TIM_TRGOSource_Enable: The Counter Enable CEN is used as the trigger output(TRGO2)
  *            @arg TIM_TRGOSource_Update: The update event is selected as the trigger output(TRGO2)
  *            @arg TIM_TRGOSource_OC1: The trigger output sends a positive pulse when the CC1IF flag
  *                                     is to be set, as soon as a capture or compare match occurs(TRGO2)
  *            @arg TIM_TRGOSource_OC1Ref: OC1REF signal is used as the trigger output(TRGO2)
  *            @arg TIM_TRGOSource_OC2Ref: OC2REF signal is used as the trigger output(TRGO2)
  *            @arg TIM_TRGOSource_OC3Ref: OC3REF signal is used as the trigger output(TRGO2)
  *            @arg TIM_TRGOSource_OC4Ref: OC4REF signal is used as the trigger output(TRGO2)
  *            @arg TIM_TRGO2Source_OC4Ref_RisingFalling: OC4Ref Rising and Falling are used as the trigger output(TRGO2)
  *            @arg TIM_TRGO2Source_OC6Ref_RisingFalling: OC6Ref Rising and Falling are used as the trigger output(TRGO2)
  *            @arg TIM_TRGO2Source_OC4RefRising_OC6RefRising: OC4Ref Rising and OC6Ref Rising  are used as the trigger output(TRGO2)
  *            @arg TIM_TRGO2Source_OC4RefRising_OC6RefFalling: OC4Ref Rising and OC6Ref Falling are used as the trigger output(TRGO2)
  *            @arg TIM_TRGO2Source_OC5RefRising_OC6RefRising: OC5Ref Rising and OC6Ref Rising are used as the trigger output(TRGO2)
  *            @arg TIM_TRGO2Source_OC5RefRising_OC6RefFalling: OC5Ref Rising and OC6Ref Falling are used as the trigger output(TRGO2)
  *
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectOutputTrigger2(mut TIMx: *mut TIM_TypeDef,
                                                  mut TIM_TRGO2Source:
                                                      uint32_t) {
    /* Check the parameters */
    /* Reset the MMS Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0xf00000i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    /* Select the TRGO source */
    ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | TIM_TRGO2Source) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Selects the TIMx Slave Mode.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM peripheral.
  * @param  TIM_SlaveMode: specifies the Timer Slave Mode.
  *          This parameter can be one of the following values:
  *            @arg TIM_SlaveMode_Reset: Rising edge of the selected trigger signal(TRGI) reinitialize 
  *                                      the counter and triggers an update of the registers
  *            @arg TIM_SlaveMode_Gated:     The counter clock is enabled when the trigger signal (TRGI) is high
  *            @arg TIM_SlaveMode_Trigger:   The counter starts at a rising edge of the trigger TRGI
  *            @arg TIM_SlaveMode_External1: Rising edges of the selected trigger (TRGI) clock the counter
  *            @arg TIM_SlaveMode_Combined_ResetTrigger: Rising edge of the selected trigger input (TRGI)
  *                                                      reinitializes the counter, generates an update 
  *                                                      of the registers and starts the counter.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectSlaveMode(mut TIMx: *mut TIM_TypeDef,
                                             mut TIM_SlaveMode: uint32_t) {
    /* Check the parameters */
    /* Reset the SMS Bits */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x10007i32 as uint32_t)) as uint32_t as
                                    uint32_t);
    /* Select the Slave Mode */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | TIM_SlaveMode) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Sets or Resets the TIMx Master/Slave Mode.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM peripheral.
  * @param  TIM_MasterSlaveMode: specifies the Timer Master Slave Mode.
  *          This parameter can be one of the following values:
  *            @arg TIM_MasterSlaveMode_Enable: synchronization between the current timer
  *                                             and its slaves (through TRGO)
  *            @arg TIM_MasterSlaveMode_Disable: No action
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectMasterSlaveMode(mut TIMx: *mut TIM_TypeDef,
                                                   mut TIM_MasterSlaveMode:
                                                       uint16_t) {
    /* Check the parameters */
    /* Reset the MSM Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x80i32 as uint32_t) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    /* Set or Reset the MSM Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).SMCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TIM_MasterSlaveMode as libc::c_uint) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Configures the TIMx External Trigger (ETR).
  * @param  TIMx: where x can be  1, 2, 3, 4 or 8 to select the TIM peripheral.
  * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
  *            @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
  *            @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
  *            @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
  * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
  *            @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
  * @param  ExtTRGFilter: External Trigger Filter.
  *          This parameter must be a value between 0x00 and 0x0F
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_ETRConfig(mut TIMx: *mut TIM_TypeDef,
                                       mut TIM_ExtTRGPrescaler: uint16_t,
                                       mut TIM_ExtTRGPolarity: uint16_t,
                                       mut ExtTRGFilter: uint16_t) {
    let mut tmpsmcr: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    tmpsmcr = (*TIMx).SMCR as uint16_t;
    /* Reset the ETR Bits */
    tmpsmcr =
        (tmpsmcr as libc::c_int & 0xffi32 as uint16_t as libc::c_int) as
            uint16_t;
    /* Set the Prescaler, the Filter value and the Polarity */
    tmpsmcr =
        (tmpsmcr as libc::c_int |
             (TIM_ExtTRGPrescaler as libc::c_int |
                  (TIM_ExtTRGPolarity as libc::c_int |
                       ((ExtTRGFilter as libc::c_int) <<
                            8i32 as uint16_t as libc::c_int) as uint16_t as
                           libc::c_int) as uint16_t as libc::c_int) as
                 uint16_t as libc::c_int) as uint16_t;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                tmpsmcr as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup TIM_Group8 Specific interface management functions
 *  @brief    Specific interface management functions 
 *
@verbatim   
 ===============================================================================
              ##### Specific interface management functions #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the TIMx Encoder Interface.
  * @param  TIMx: where x can be 1, 2, 3, 4 or 8 to select the TIM 
  *         peripheral.
  * @param  TIM_EncoderMode: specifies the TIMx Encoder Mode.
  *          This parameter can be one of the following values:
  *            @arg TIM_EncoderMode_TI1: Counter counts on TI1FP1 edge depending on TI2FP2 level.
  *            @arg TIM_EncoderMode_TI2: Counter counts on TI2FP2 edge depending on TI1FP1 level.
  *            @arg TIM_EncoderMode_TI12: Counter counts on both TI1FP1 and TI2FP2 edges depending
  *                                       on the level of the other input.
  * @param  TIM_IC1Polarity: specifies the IC1 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Falling: IC Falling edge.
  *            @arg TIM_ICPolarity_Rising: IC Rising edge.
  * @param  TIM_IC2Polarity: specifies the IC2 Polarity
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Falling: IC Falling edge.
  *            @arg TIM_ICPolarity_Rising: IC Rising edge.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_EncoderInterfaceConfig(mut TIMx:
                                                        *mut TIM_TypeDef,
                                                    mut TIM_EncoderMode:
                                                        uint16_t,
                                                    mut TIM_IC1Polarity:
                                                        uint16_t,
                                                    mut TIM_IC2Polarity:
                                                        uint16_t) {
    let mut tmpsmcr: uint16_t = 0i32 as uint16_t;
    let mut tmpccmr1: uint16_t = 0i32 as uint16_t;
    let mut tmpccer: uint16_t = 0i32 as uint16_t;
    /* Check the parameters */
    /* Get the TIMx SMCR register value */
    tmpsmcr = (*TIMx).SMCR as uint16_t;
    /* Get the TIMx CCMR1 register value */
    tmpccmr1 = (*TIMx).CCMR1 as uint16_t;
    /* Get the TIMx CCER register value */
    tmpccer = (*TIMx).CCER as uint16_t;
    /* Set the encoder Mode */
    tmpsmcr =
        (tmpsmcr as libc::c_int &
             !(0x10007i32 as uint32_t) as uint16_t as libc::c_int) as
            uint16_t;
    tmpsmcr =
        (tmpsmcr as libc::c_int | TIM_EncoderMode as libc::c_int) as uint16_t;
    /* Select the Capture Compare 1 and the Capture Compare 2 as input */
    tmpccmr1 =
        (tmpccmr1 as libc::c_int &
             (!(0x3i32 as uint32_t) as uint16_t as libc::c_int &
                  !(0x300i32 as uint32_t) as uint16_t as libc::c_int)) as
            uint16_t;
    tmpccmr1 =
        (tmpccmr1 as libc::c_uint |
             (0x1i32 as uint32_t | 0x100i32 as uint32_t)) as uint16_t;
    /* Set the TI1 and the TI2 Polarities */
    tmpccer =
        (tmpccer as libc::c_int &
             (!(0x2i32 as uint32_t) as uint16_t as libc::c_int &
                  !(0x20i32 as uint32_t) as uint16_t as libc::c_int)) as
            uint16_t;
    tmpccer =
        (tmpccer as libc::c_int |
             (TIM_IC1Polarity as libc::c_int |
                  ((TIM_IC2Polarity as libc::c_int) <<
                       4i32 as uint16_t as libc::c_int) as uint16_t as
                      libc::c_int) as uint16_t as libc::c_int) as uint16_t;
    /* Write to TIMx SMCR */
    ::core::ptr::write_volatile(&mut (*TIMx).SMCR as *mut uint32_t,
                                tmpsmcr as uint32_t);
    /* Write to TIMx CCMR1 */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1 as uint32_t);
    /* Write to TIMx CCER */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                tmpccer as uint32_t);
}
/* *
  * @brief  Enables or disables the TIMx's Hall sensor interface.
  * @param  TIMx: where x can be 1, 2, 3, 4, 8 or 15 to select the TIM 
  *         peripheral.
  * @param  NewState: new state of the TIMx Hall sensor interface.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_SelectHallSensor(mut TIMx: *mut TIM_TypeDef,
                                              mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        /* Set the TI1S Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x80i32 as uint32_t) as uint32_t as
                                        uint32_t)
    } else {
        /* Reset the TI1S Bit */
        ::core::ptr::write_volatile(&mut (*TIMx).CR2 as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CR2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x80i32 as uint32_t) as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t)
    };
}
/* *
  ******************************************************************************
  * @file    stm32f30x_tim.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the TIM firmware 
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
/* * @addtogroup stm32f30x_StdPeriph_Driver
  * @{
  */
/* * @addtogroup TIM
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  TIM Time Base Init structure definition  
  * @note   This structure is used with all TIMx except for TIM6 and TIM7.  
  */
/* !< Specifies the prescaler value used to divide the TIM clock.
                                       This parameter can be a number between 0x0000 and 0xFFFF */
/* !< Specifies the counter mode.
                                       This parameter can be a value of @ref TIM_Counter_Mode */
/* !< Specifies the period value to be loaded into the active
                                       Auto-Reload Register at the next update event.
                                       This parameter must be a number between 0x0000 and 0xFFFF.  */
/* !< Specifies the clock division.
                                      This parameter can be a value of @ref TIM_Clock_Division_CKD */
/* !< Specifies the repetition counter value. Each time the RCR downcounter
                                       reaches zero, an update event is generated and counting restarts
                                       from the RCR value (N).
                                       This means in PWM mode that (N+1) corresponds to:
                                          - the number of PWM periods in edge-aligned mode
                                          - the number of half PWM period in center-aligned mode
                                       This parameter must be a number between 0x00 and 0xFF. 
                                       @note This parameter is valid only for TIM1 and TIM8. */
/* * 
  * @brief  TIM Output Compare Init structure definition  
  */
/* !< Specifies the TIM mode.
                                   This parameter can be a value of @ref TIM_Output_Compare_and_PWM_modes */
/* !< Specifies the TIM Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_State */
/* !< Specifies the TIM complementary Output Compare state.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_State
                                   @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the pulse value to be loaded into the Capture Compare Register. 
                                   This parameter can be a number between 0x0000 and 0xFFFF */
/* !< Specifies the output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_Polarity */
/* !< Specifies the complementary output polarity.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_Polarity
                                   @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the TIM Output Compare pin state during Idle state.
                                   This parameter can be a value of @ref TIM_Output_Compare_Idle_State
                                   @note This parameter is valid only for TIM1 and TIM8. */
/* !< Specifies the TIM Output Compare pin state during Idle state.
                                   This parameter can be a value of @ref TIM_Output_Compare_N_Idle_State
                                   @note This parameter is valid only for TIM1 and TIM8. */
/* * 
  * @brief  TIM Input Capture Init structure definition  
  */
/* !< Specifies the TIM channel.
                                  This parameter can be a value of @ref TIM_Channel */
/* !< Specifies the active edge of the input signal.
                                  This parameter can be a value of @ref TIM_Input_Capture_Polarity */
/* !< Specifies the input.
                                  This parameter can be a value of @ref TIM_Input_Capture_Selection */
/* !< Specifies the Input Capture Prescaler.
                                  This parameter can be a value of @ref TIM_Input_Capture_Prescaler */
/* !< Specifies the input capture filter.
                                  This parameter can be a number between 0x0 and 0xF */
/* * 
  * @brief  BDTR structure definition 
  * @note   This structure is used only with TIM1 and TIM8.    
  */
/* !< Specifies the Off-State selection used in Run mode.
                                      This parameter can be a value of @ref TIM_OSSR_Off_State_Selection_for_Run_mode_state */
/* !< Specifies the Off-State used in Idle state.
                                      This parameter can be a value of @ref TIM_OSSI_Off_State_Selection_for_Idle_mode_state */
/* !< Specifies the LOCK level parameters.
                                      This parameter can be a value of @ref TIM_Lock_level */
/* !< Specifies the delay time between the switching-off and the
                                      switching-on of the outputs.
                                      This parameter can be a number between 0x00 and 0xFF  */
/* !< Specifies whether the TIM Break input is enabled or not. 
                                      This parameter can be a value of @ref TIM_Break_Input_enable_disable */
/* !< Specifies the TIM Break Input pin polarity.
                                      This parameter can be a value of @ref TIM_Break_Polarity */
/* !< Specifies whether the TIM Automatic Output feature is enabled or not. 
                                      This parameter can be a value of @ref TIM_AOE_Bit_Set_Reset */
/* Exported constants --------------------------------------------------------*/
/* * @defgroup TIM_Exported_constants 
  * @{
  */
/* LIST1: TIM1, TIM2, TIM3, TIM4, TIM8, TIM15, TIM16 and TIM17 */
/* LIST2: TIM1, TIM2, TIM3, TIM4, TIM8 and TIM15 */
/* LIST3: TIM1, TIM2, TIM3, TIM4 and TIM8 */
/* LIST4: TIM1 and TIM8 */
/* LIST5: TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7 and TIM8 */
/* LIST6: TIM1, TIM8, TIM15, TIM16 and TIM17 */
/* LIST5: TIM1, TIM2, TIM3, TIM4, TIM5, TIM6, TIM7 and TIM8 */
/* LIST8: TIM16 (option register) */
/* * @defgroup TIM_Output_Compare_and_PWM_modes 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_One_Pulse_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Channel 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Clock_Division_CKD 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Counter_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_N_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_N_State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Capture_Compare_State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Capture_Compare_N_State
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break_Input_enable_disable 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break1_Input_enable_disable 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break2_Input_enable_disable 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break1_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break2_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break1_Filter 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Break2_Filter 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_AOE_Bit_Set_Reset 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Lock_level
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_OSSI_Off_State_Selection_for_Idle_mode_state 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_OSSR_Off_State_Selection_for_Run_mode_state
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Idle_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_N_Idle_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Input_Capture_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Input_Capture_Selection 
  * @{
  */
/* !< TIM Input 1, 2, 3 or 4 is selected to be 
                                                                   connected to IC1, IC2, IC3 or IC4, respectively */
/* !< TIM Input 1, 2, 3 or 4 is selected to be
                                                                   connected to IC2, IC1, IC4 or IC3, respectively. */
/* !< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC. */
/* *
  * @}
  */
/* * @defgroup TIM_Input_Capture_Prescaler 
  * @{
  */
/* !< Capture performed each time an edge is detected on the capture input. */
/* !< Capture performed once every 2 events. */
/* !< Capture performed once every 4 events. */
/* !< Capture performed once every 8 events. */
/* *
  * @}
  */
/* * @defgroup TIM_interrupt_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_DMA_Base_address 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_DMA_Burst_Length 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_DMA_sources 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_External_Trigger_Prescaler 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Internal_Trigger_Selection 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_TIx_External_Clock_Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_External_Trigger_Polarity 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Prescaler_Reload_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Forced_Action 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Encoder_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Event_Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Update_Source 
  * @{
  */
/* !< Source of update is the counter overflow/underflow
                                                                   or the setting of UG bit, or an update generation
                                                                   through the slave mode controller. */
/* !< Source of update is counter overflow/underflow. */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Preload_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Fast_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Output_Compare_Clear_State 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Trigger_Output_Source 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Slave_Mode 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Master_Slave_Mode 
  * @{
  */
/* *
  * @}
  */ 
/* * @defgroup TIM_Remap 
  * @{
  */
/* *
  * @}
  */ 
/* * @defgroup TIM_Flags 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_OCReferenceClear 
  * @{
  */
/* * @defgroup TIM_Input_Capture_Filer_Value 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_External_Trigger_Filter 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup TIM_Legacy 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* TimeBase management ********************************************************/
/* Output Compare management **************************************************/
/* Input Capture management ***************************************************/
/* Advanced-control timers (TIM1 and TIM8) specific features ******************/
/* Interrupts, DMA and flags management ***************************************/
/* Clocks management **********************************************************/
/* Synchronization management *************************************************/
/* Specific interface management **********************************************/
/* Specific remapping management **********************************************/
/* *
  * @}
  */
/* * @defgroup TIM_Group9 Specific remapping management function
 *  @brief   Specific remapping management function
 *
@verbatim   
 ===============================================================================
               ##### Specific remapping management function #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Configures the TIM16 Remapping input Capabilities.
  * @param TIMx: where x can be 1, 8 or 16 to select the TIM peripheral.
  * @param TIM_Remap: specifies the TIM input reampping source.
  *   This parameter can be one of the following values:
  *            @arg TIM16_GPIO: TIM16 Channel 1 is connected to GPIO.
  *            @arg TIM16_RTC_CLK: TIM16 Channel 1 is connected to RTC input clock.
  *            @arg TIM16_HSE_DIV32: TIM16 Channel 1 is connected to HSE/32 clock.  
  *            @arg TIM16_MCO: TIM16 Channel 1 is connected to MCO clock. 
  *            @arg TIM1_ADC1_AWDG1: TIM1 ETR is connected to ADC1 AWDG1.
  *            @arg TIM1_ADC1_AWDG2: TIM1 ETR is connected to ADC1 AWDG2.
  *            @arg TIM1_ADC1_AWDG3: TIM1 ETR is connected to ADC1 AWDG3.  
  *            @arg TIM1_ADC4_AWDG1: TIM1 ETR is connected to ADC4 AWDG1.
  *            @arg TIM1_ADC4_AWDG2: TIM1 ETR is connected to ADC4 AWDG2.
  *            @arg TIM1_ADC4_AWDG3: TIM1 ETR is connected to ADC4 AWDG3. 
  *            @arg TIM8_ADC2_AWDG1: TIM8 ETR is connected to ADC2 AWDG1.
  *            @arg TIM8_ADC2_AWDG2: TIM8 ETR is connected to ADC2 AWDG2.
  *            @arg TIM8_ADC2_AWDG3: TIM8 ETR is connected to ADC2 AWDG3.
  *            @arg TIM8_ADC4_AWDG1: TIM8 ETR is connected to ADC4 AWDG1.
  *            @arg TIM8_ADC4_AWDG2: TIM8 ETR is connected to ADC4 AWDG2.
  *            @arg TIM8_ADC4_AWDG3: TIM8 ETR is connected to ADC4 AWDG3.  
  * @retval : None
  */
#[no_mangle]
pub unsafe extern "C" fn TIM_RemapConfig(mut TIMx: *mut TIM_TypeDef,
                                         mut TIM_Remap: uint16_t) {
    /* Check the parameters */
    /* Set the Timer remapping configuration */
    ::core::ptr::write_volatile(&mut (*TIMx).OR as *mut uint16_t, TIM_Remap);
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* *
  * @}
  */
/* *
  * @brief  Configure the TI1 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13 or 14 
  *         to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge  
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 1 is selected to be connected to IC1.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 1 is selected to be connected to IC2.
  *            @arg TIM_ICSelection_TRC: TIM Input 1 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
unsafe extern "C" fn TI1_Config(mut TIMx: *mut TIM_TypeDef,
                                mut TIM_ICPolarity: uint16_t,
                                mut TIM_ICSelection: uint16_t,
                                mut TIM_ICFilter: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    /* Disable the Channel 1: Reset the CC1E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x1i32 as uint32_t))
                                    as uint32_t as uint32_t);
    tmpccmr1 = (*TIMx).CCMR1;
    tmpccer = (*TIMx).CCER;
    /* Select the Input and set the filter */
    tmpccmr1 &= !(0x3i32 as uint32_t) & !(0xf0i32 as uint32_t);
    tmpccmr1 |=
        TIM_ICSelection as libc::c_uint | (TIM_ICFilter as uint32_t) << 4i32;
    /* Select the Polarity and set the CC1E Bit */
    tmpccer &= !(0x2i32 as uint32_t | 0x8i32 as uint32_t);
    tmpccer |= TIM_ICPolarity as libc::c_uint | 0x1i32 as uint32_t;
    /* Write to TIMx CCMR1 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configure the TI2 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5, 8, 9 or 12 to select the TIM 
  *         peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge   
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 2 is selected to be connected to IC2.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 2 is selected to be connected to IC1.
  *            @arg TIM_ICSelection_TRC: TIM Input 2 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
unsafe extern "C" fn TI2_Config(mut TIMx: *mut TIM_TypeDef,
                                mut TIM_ICPolarity: uint16_t,
                                mut TIM_ICSelection: uint16_t,
                                mut TIM_ICFilter: uint16_t) {
    let mut tmpccmr1: uint32_t = 0i32 as uint32_t;
    let mut tmpccer: uint32_t = 0i32 as uint32_t;
    let mut tmp: uint32_t = 0i32 as uint32_t;
    /* Disable the Channel 2: Reset the CC2E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x10i32 as uint32_t) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    tmpccmr1 = (*TIMx).CCMR1;
    tmpccer = (*TIMx).CCER;
    tmp = ((TIM_ICPolarity as libc::c_int) << 4i32) as uint16_t as uint32_t;
    /* Select the Input and set the filter */
    tmpccmr1 &= !(0x300i32 as uint32_t) & !(0xf000i32 as uint32_t);
    tmpccmr1 |= (TIM_ICFilter as uint32_t) << 12i32;
    tmpccmr1 |= (TIM_ICSelection as uint32_t) << 8i32;
    /* Select the Polarity and set the CC2E Bit */
    tmpccer &=
        !(0x20i32 as uint32_t | 0x80i32 as uint32_t) as uint16_t as
            libc::c_uint;
    tmpccer |=
        (tmp | 0x10i32 as uint32_t as uint16_t as libc::c_uint) as uint16_t as
            libc::c_uint;
    /* Write to TIMx CCMR1 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR1 as *mut uint32_t,
                                tmpccmr1);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t, tmpccer);
}
/* *
  * @brief  Configure the TI3 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge         
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 3 is selected to be connected to IC3.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 3 is selected to be connected to IC4.
  *            @arg TIM_ICSelection_TRC: TIM Input 3 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
unsafe extern "C" fn TI3_Config(mut TIMx: *mut TIM_TypeDef,
                                mut TIM_ICPolarity: uint16_t,
                                mut TIM_ICSelection: uint16_t,
                                mut TIM_ICFilter: uint16_t) {
    let mut tmpccmr2: uint16_t = 0i32 as uint16_t;
    let mut tmpccer: uint16_t = 0i32 as uint16_t;
    let mut tmp: uint16_t = 0i32 as uint16_t;
    /* Disable the Channel 3: Reset the CC3E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x100i32 as uint32_t) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    tmpccmr2 = (*TIMx).CCMR2 as uint16_t;
    tmpccer = (*TIMx).CCER as uint16_t;
    tmp = ((TIM_ICPolarity as libc::c_int) << 8i32) as uint16_t;
    /* Select the Input and set the filter */
    tmpccmr2 =
        (tmpccmr2 as libc::c_int &
             (!(0x3i32 as uint32_t) as uint16_t as libc::c_int &
                  !(0xf0i32 as uint16_t as libc::c_int) as uint16_t as
                      libc::c_int)) as uint16_t;
    tmpccmr2 =
        (tmpccmr2 as libc::c_int |
             (TIM_ICSelection as libc::c_int |
                  ((TIM_ICFilter as libc::c_int) <<
                       4i32 as uint16_t as libc::c_int) as uint16_t as
                      libc::c_int) as uint16_t as libc::c_int) as uint16_t;
    /* Select the Polarity and set the CC3E Bit */
    tmpccer =
        (tmpccer as libc::c_int &
             !(0x200i32 as uint32_t | 0x800i32 as uint32_t) as uint16_t as
                 libc::c_int) as uint16_t;
    tmpccer =
        (tmpccer as libc::c_int |
             (tmp as libc::c_int |
                  0x100i32 as uint32_t as uint16_t as libc::c_int) as uint16_t
                 as libc::c_int) as uint16_t;
    /* Write to TIMx CCMR2 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2 as uint32_t);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                tmpccer as uint32_t);
}
/* *
  * @brief  Configure the TI4 as Input.
  * @param  TIMx: where x can be 1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * @param  TIM_ICPolarity : The Input Polarity.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICPolarity_Rising
  *            @arg TIM_ICPolarity_Falling
  *            @arg TIM_ICPolarity_BothEdge     
  * @param  TIM_ICSelection: specifies the input to be used.
  *          This parameter can be one of the following values:
  *            @arg TIM_ICSelection_DirectTI: TIM Input 4 is selected to be connected to IC4.
  *            @arg TIM_ICSelection_IndirectTI: TIM Input 4 is selected to be connected to IC3.
  *            @arg TIM_ICSelection_TRC: TIM Input 4 is selected to be connected to TRC.
  * @param  TIM_ICFilter: Specifies the Input Capture Filter.
  *          This parameter must be a value between 0x00 and 0x0F.
  * @retval None
  */
unsafe extern "C" fn TI4_Config(mut TIMx: *mut TIM_TypeDef,
                                mut TIM_ICPolarity: uint16_t,
                                mut TIM_ICSelection: uint16_t,
                                mut TIM_ICFilter: uint16_t) {
    let mut tmpccmr2: uint16_t = 0i32 as uint16_t;
    let mut tmpccer: uint16_t = 0i32 as uint16_t;
    let mut tmp: uint16_t = 0i32 as uint16_t;
    /* Disable the Channel 4: Reset the CC4E Bit */
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*TIMx).CCER
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x1000i32 as uint32_t) as uint16_t as
                                         libc::c_uint) as uint32_t as
                                    uint32_t);
    tmpccmr2 = (*TIMx).CCMR2 as uint16_t;
    tmpccer = (*TIMx).CCER as uint16_t;
    tmp = ((TIM_ICPolarity as libc::c_int) << 12i32) as uint16_t;
    /* Select the Input and set the filter */
    tmpccmr2 =
        (tmpccmr2 as libc::c_int &
             (!(0x300i32 as uint32_t) as uint16_t as libc::c_int &
                  !(0xf000i32 as uint32_t) as uint16_t as libc::c_int)) as
            uint16_t;
    tmpccmr2 =
        (tmpccmr2 as libc::c_int |
             ((TIM_ICSelection as libc::c_int) << 8i32) as uint16_t as
                 libc::c_int) as uint16_t;
    tmpccmr2 =
        (tmpccmr2 as libc::c_int |
             ((TIM_ICFilter as libc::c_int) << 12i32) as uint16_t as
                 libc::c_int) as uint16_t;
    /* Select the Polarity and set the CC4E Bit */
    tmpccer =
        (tmpccer as libc::c_int &
             !(0x2000i32 as uint32_t | 0x8000i32 as uint32_t) as uint16_t as
                 libc::c_int) as uint16_t;
    tmpccer =
        (tmpccer as libc::c_int |
             (tmp as libc::c_int |
                  0x1000i32 as uint32_t as uint16_t as libc::c_int) as
                 uint16_t as libc::c_int) as uint16_t;
    /* Write to TIMx CCMR2 and CCER registers */
    ::core::ptr::write_volatile(&mut (*TIMx).CCMR2 as *mut uint32_t,
                                tmpccmr2 as uint32_t);
    ::core::ptr::write_volatile(&mut (*TIMx).CCER as *mut uint32_t,
                                tmpccer as uint32_t);
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
