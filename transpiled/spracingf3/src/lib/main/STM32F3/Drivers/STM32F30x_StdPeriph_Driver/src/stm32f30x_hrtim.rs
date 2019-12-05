use core;
use libc;
extern "C" {
    #[no_mangle]
    fn RCC_APB2PeriphResetCmd(RCC_APB2Periph: uint32_t,
                              NewState: FunctionalState);
}
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
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
  * @brief High resolution Timer (HRTIM)
  */
/* HRTIM master definition */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_Master_TypeDef {
    pub MCR: uint32_t,
    pub MISR: uint32_t,
    pub MICR: uint32_t,
    pub MDIER: uint32_t,
    pub MCNTR: uint32_t,
    pub MPER: uint32_t,
    pub MREP: uint32_t,
    pub MCMP1R: uint32_t,
    pub RESERVED0: uint32_t,
    pub MCMP2R: uint32_t,
    pub MCMP3R: uint32_t,
    pub MCMP4R: uint32_t,
    /* !< HRTIM Master Timer compare 4 register,                   Address offset: 0x2C */
}
/* HRTIM slave definition */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_Timerx_TypeDef {
    pub TIMxCR: uint32_t,
    pub TIMxISR: uint32_t,
    pub TIMxICR: uint32_t,
    pub TIMxDIER: uint32_t,
    pub CNTxR: uint32_t,
    pub PERxR: uint32_t,
    pub REPxR: uint32_t,
    pub CMP1xR: uint32_t,
    pub CMP1CxR: uint32_t,
    pub CMP2xR: uint32_t,
    pub CMP3xR: uint32_t,
    pub CMP4xR: uint32_t,
    pub CPT1xR: uint32_t,
    pub CPT2xR: uint32_t,
    pub DTxR: uint32_t,
    pub SETx1R: uint32_t,
    pub RSTx1R: uint32_t,
    pub SETx2R: uint32_t,
    pub RSTx2R: uint32_t,
    pub EEFxR1: uint32_t,
    pub EEFxR2: uint32_t,
    pub RSTxR: uint32_t,
    pub CHPxR: uint32_t,
    pub CPT1xCR: uint32_t,
    pub CPT2xCR: uint32_t,
    pub OUTxR: uint32_t,
    pub FLTxR: uint32_t,
    pub RESERVED0: [uint32_t; 5],
    /* !< Reserved,                                                                       */
}
/* HRTIM common register definition */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_Common_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub IER: uint32_t,
    pub OENR: uint32_t,
    pub DISR: uint32_t,
    pub ODSR: uint32_t,
    pub BMCR: uint32_t,
    pub BMTRGR: uint32_t,
    pub BMCMPR: uint32_t,
    pub BMPER: uint32_t,
    pub EECR1: uint32_t,
    pub EECR2: uint32_t,
    pub EECR3: uint32_t,
    pub ADC1R: uint32_t,
    pub ADC2R: uint32_t,
    pub ADC3R: uint32_t,
    pub ADC4R: uint32_t,
    pub DLLCR: uint32_t,
    pub FLTINxR1: uint32_t,
    pub FLTINxR2: uint32_t,
    pub BDMUPDR: uint32_t,
    pub BDTAUPR: uint32_t,
    pub BDTBUPR: uint32_t,
    pub BDTCUPR: uint32_t,
    pub BDTDUPR: uint32_t,
    pub BDTEUPR: uint32_t,
    pub BDMADR: uint32_t,
    /* !< HRTIM Burst DMA Master Data register,                       Address offset: 0x70 */
}
/* HRTIM  register definition */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_TypeDef {
    pub HRTIM_MASTER: HRTIM_Master_TypeDef,
    pub RESERVED0: [uint32_t; 20],
    pub HRTIM_TIMERx: [HRTIM_Timerx_TypeDef; 5],
    pub RESERVED1: [uint32_t; 32],
    pub HRTIM_COMMON: HRTIM_Common_TypeDef,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_BaseInitTypeDef {
    pub Period: uint32_t,
    pub RepetitionCounter: uint32_t,
    pub PrescalerRatio: uint32_t,
    pub Mode: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_TimerInitTypeDef {
    pub HalfModeEnable: uint32_t,
    pub StartOnSync: uint32_t,
    pub ResetOnSync: uint32_t,
    pub DACSynchro: uint32_t,
    pub PreloadEnable: uint32_t,
    pub UpdateGating: uint32_t,
    pub BurstMode: uint32_t,
    pub RepetitionUpdate: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_BasicOCChannelCfgTypeDef {
    pub Mode: uint32_t,
    pub Pulse: uint32_t,
    pub Polarity: uint32_t,
    pub IdleState: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_BasicPWMChannelCfgTypeDef {
    pub Pulse: uint32_t,
    pub Polarity: uint32_t,
    pub IdleState: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_BasicCaptureChannelCfgTypeDef {
    pub CaptureUnit: uint32_t,
    pub Event: uint32_t,
    pub EventPolarity: uint32_t,
    pub EventSensitivity: uint32_t,
    pub EventFilter: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_BasicOnePulseChannelCfgTypeDef {
    pub Pulse: uint32_t,
    pub OutputPolarity: uint32_t,
    pub OutputIdleState: uint32_t,
    pub Event: uint32_t,
    pub EventPolarity: uint32_t,
    pub EventSensitivity: uint32_t,
    pub EventFilter: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_TimerCfgTypeDef {
    pub PushPull: uint32_t,
    pub FaultEnable: uint32_t,
    pub FaultLock: uint32_t,
    pub DeadTimeInsertion: uint32_t,
    pub DelayedProtectionMode: uint32_t,
    pub UpdateTrigger: uint32_t,
    pub ResetTrigger: uint32_t,
    pub ResetUpdate: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_CompareCfgTypeDef {
    pub CompareValue: uint32_t,
    pub AutoDelayedMode: uint32_t,
    pub AutoDelayedTimeout: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_CaptureCfgTypeDef {
    pub Trigger: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_OutputCfgTypeDef {
    pub Polarity: uint32_t,
    pub SetSource: uint32_t,
    pub ResetSource: uint32_t,
    pub IdleMode: uint32_t,
    pub IdleState: uint32_t,
    pub FaultState: uint32_t,
    pub ChopperModeEnable: uint32_t,
    pub BurstModeEntryDelayed: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_TimerEventFilteringCfgTypeDef {
    pub Filter: uint32_t,
    pub Latch: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_DeadTimeCfgTypeDef {
    pub Prescaler: uint32_t,
    pub RisingValue: uint32_t,
    pub RisingSign: uint32_t,
    pub RisingLock: uint32_t,
    pub RisingSignLock: uint32_t,
    pub FallingValue: uint32_t,
    pub FallingSign: uint32_t,
    pub FallingLock: uint32_t,
    pub FallingSignLock: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_ChopperModeCfgTypeDef {
    pub CarrierFreq: uint32_t,
    pub DutyCycle: uint32_t,
    pub StartPulse: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_SynchroCfgTypeDef {
    pub SyncInputSource: uint32_t,
    pub SyncOutputSource: uint32_t,
    pub SyncOutputPolarity: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_EventCfgTypeDef {
    pub Source: uint32_t,
    pub Polarity: uint32_t,
    pub Sensitivity: uint32_t,
    pub Filter: uint32_t,
    pub FastMode: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_FaultCfgTypeDef {
    pub Source: uint32_t,
    pub Polarity: uint32_t,
    pub Filter: uint32_t,
    pub Lock: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_BurstModeCfgTypeDef {
    pub Mode: uint32_t,
    pub ClockSource: uint32_t,
    pub Prescaler: uint32_t,
    pub PreloadEnable: uint32_t,
    pub Trigger: uint32_t,
    pub IdleDuration: uint32_t,
    pub Period: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct HRTIM_ADCTriggerCfgTypeDef {
    pub UpdateSource: uint32_t,
    pub Trigger: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static mut TimerIdxToTimerId: [uint32_t; 6] =
    [0x20000i32 as uint32_t, 0x40000i32 as uint32_t, 0x80000i32 as uint32_t,
     0x100000i32 as uint32_t, 0x200000i32 as uint32_t,
     0x10000i32 as uint32_t];
/* * @defgroup HRTIM_Private_Functions
  * @{
  */
/* * @defgroup HRTIM_Group1 Initialization/de-initialization methods 
 *  @brief    Initialization and Configuration functions 
 *
@verbatim    
 ===============================================================================
              ##### Initialization/de-initialization methods #####
 ===============================================================================
    [..]  This section provides functions allowing to:
          (+)Initializes timer in basic time base mode
          (+)Initializes timer in basic OC mode
          (+)Initializes timer in basic PWM mode
          (+)Initializes timer in basic Capture mode
          (+)Initializes timer in One Pulse mode
          (+)Initializes a timer operating in waveform mode
          (+)De-initializes the HRTIMx timer
 
@endverbatim
  * @{
  */
/* *
  * @brief  Initializes the HRTIMx timer in basic time base mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 for master timer
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   The time-base unit initialization parameters specify:
  *           The timer counter operating mode (continuous, one shot)
  *           The timer clock prescaler
  *           The timer period 
  *           The timer repetition counter.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleBase_Init(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut TimerIdx: uint32_t,
                                               mut HRTIM_BaseInitStruct:
                                                   *mut HRTIM_BaseInitTypeDef) {
    /* Check the parameters */
    if TimerIdx == 0x5i32 as uint32_t {
        /* Configure master timer */
        HRTIM_MasterBase_Config(HRTIMx, HRTIM_BaseInitStruct);
    } else {
        /* Configure timing unit */
        HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
    };
}
/* *
  * @brief  De-initializes a timer operating in all mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_DeInit(mut HRTIMx: *mut HRTIM_TypeDef) {
    /* Check the parameters */
    RCC_APB2PeriphResetCmd(0x20000000i32 as uint32_t, ENABLE);
    RCC_APB2PeriphResetCmd(0x20000000i32 as uint32_t, DISABLE);
}
/* *
  * @brief  Initializes the HRTIMx timer in basic output compare mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   Initializes the time-base unit of the timer and prepare it to
  *         operate in output compare mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOC_Init(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut HRTIM_BaseInitStruct:
                                                 *mut HRTIM_BaseInitTypeDef) {
    /* Check the parameters */
    /* Configure timing unit */
    HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}
/* *
  * @brief  Initializes the HRTIMx timer in basic PWM mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   Initializes the time-base unit of the timer and prepare it to
  *         operate in capture mode
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimplePWM_Init(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut TimerIdx: uint32_t,
                                              mut HRTIM_BaseInitStruct:
                                                  *mut HRTIM_BaseInitTypeDef) {
    /* Check the parameters */
    /* Configure timing unit */
    HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}
/* *
  * @brief  Initializes a timer operating in basic capture mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleCapture_Init(mut HRTIMx:
                                                      *mut HRTIM_TypeDef,
                                                  mut TimerIdx: uint32_t,
                                                  mut HRTIM_BaseInitStruct:
                                                      *mut HRTIM_BaseInitTypeDef) {
    /* Check the parameters */
    /* Configure timing unit */
    HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}
/* *
  * @brief  Initializes the HRTIMx timer in basic one pulse mode 
  * @param  HRTIMx: pointer to  HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x1 to 0x5 for timers A to E
  * @note   Initializes the time-base unit of the timer and prepare it to
  *         operate in one pulse mode. In this mode the counter operates
  *         in single shot mode (retriggerable or not)
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOnePulse_Init(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut TimerIdx: uint32_t,
                                                   mut HRTIM_BaseInitStruct:
                                                       *mut HRTIM_BaseInitTypeDef) {
    /* Check the parameters */
    /* Configure timing unit */
    HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
}
/* *
  * @brief  Initializes a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 for master timer
  *                   @arg 0x1 to 0x5 for timers A to E 
  * @param  pTimerInit: pointer to the timer initialization data structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_Waveform_Init(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut HRTIM_BaseInitStruct:
                                                 *mut HRTIM_BaseInitTypeDef,
                                             mut HRTIM_TimerInitStruct:
                                                 *mut HRTIM_TimerInitTypeDef) {
    /* Check the parameters */
    if TimerIdx == 0x5i32 as uint32_t {
        /* Check parameters */
        /* Configure master timer */
        HRTIM_MasterBase_Config(HRTIMx, HRTIM_BaseInitStruct);
        HRTIM_MasterWaveform_Config(HRTIMx, HRTIM_TimerInitStruct);
    } else {
        /* Check parameters */
        /* Configure timing unit */
        HRTIM_TimingUnitBase_Config(HRTIMx, TimerIdx, HRTIM_BaseInitStruct);
        HRTIM_TimingUnitWaveform_Config(HRTIMx, TimerIdx,
                                        HRTIM_TimerInitStruct);
    };
}
/* *
  * @}
  */
/* * @defgroup HRTIM_Group2 I/O operation methods 
 *  @brief   Data transfers functions 
 *
@verbatim   
 ===============================================================================
                      ##### IO operation methods #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to manage the HRTIMx data 
    transfers.
    (+) Starts the DLL calibration.
    (+) Starts / stops the counter of a timer operating in basic time base mode
    (+) Starts / stops the output compare signal generation on the designed timer output
    (+) Starts / stops the PWM output signal generation on the designed timer output
    (+) Enables / disables a basic capture on the designed capture unit

@endverbatim
  * @{
  */
/* *
  * @brief  Starts the DLL calibration
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  CalibrationRate: DLL calibration period
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_SINGLE_CALIBRATION: One shot DLL calibration
  *                    @arg HRTIM_CALIBRATIONRATE_7300: 7.3 ms
  *                    @arg HRTIM_CALIBRATIONRATE_910: 910 us
  *                    @arg HRTIM_CALIBRATIONRATE_114: 114 us
  *                    @arg HRTIM_CALIBRATIONRATE_14: 14 us
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_DLLCalibrationStart(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut CalibrationRate:
                                                       uint32_t) {
    let mut HRTIM_dllcr: uint32_t = 0;
    /* Check the parameters */
    /* Configure DLL Calibration */
    HRTIM_dllcr = (*HRTIMx).HRTIM_COMMON.DLLCR;
    if CalibrationRate == 0xffffffffu32 {
        /* One shot DLL calibration */
        HRTIM_dllcr &= !(0x2i32 as uint32_t);
        HRTIM_dllcr |= 0x1i32 as uint32_t
    } else {
        /* Periodic DLL calibration */
        HRTIM_dllcr &= !(0xci32 as uint32_t | 0x1i32 as uint32_t);
        HRTIM_dllcr |= CalibrationRate | 0x2i32 as uint32_t
    }
    /* Update HRTIMx register */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.DLLCR as
                                    *mut uint32_t, HRTIM_dllcr);
}
/* *
  * @brief  Starts the counter of a timer operating in basic time base mode
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x5 for master timer
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleBaseStart(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut TimerIdx: uint32_t) {
    /* Check the parameters */
    /* Enable the timer counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TimerIdxToTimerId[TimerIdx as usize]) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Stops the counter of a timer operating in basic time base mode
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x5 for master timer
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleBaseStop(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut TimerIdx: uint32_t) {
    /* Check the parameters */
    /* Disable the timer counter */
    if TimerIdxToTimerId[TimerIdx as usize] & 0x10000i32 as uint32_t ==
           0x10000i32 as uint32_t {
        ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x20000i32 as uint32_t ==
           0x20000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x2i32 as uint32_t | 0x1i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x40000i32 as uint32_t ==
           0x40000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x8i32 as uint32_t | 0x4i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x40000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x80000i32 as uint32_t ==
           0x80000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x20i32 as uint32_t | 0x10i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x80000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x100000i32 as uint32_t ==
           0x100000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x80i32 as uint32_t | 0x40i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x200000i32 as uint32_t ==
           0x200000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x200i32 as uint32_t | 0x100i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x200000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    };
}
/* *
  * @brief  Starts the output compare signal generation on the designed timer output 
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OCChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOCStart(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut OCChannel: uint32_t) {
    /* Check the parameters */
    /* Enable the timer output */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.OENR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.OENR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OCChannel) as uint32_t
                                    as uint32_t);
    /* Enable the timer counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TimerIdxToTimerId[TimerIdx as usize]) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Stops the output compare signal generation on the designed timer output 
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OCChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOCStop(mut HRTIMx: *mut HRTIM_TypeDef,
                                            mut TimerIdx: uint32_t,
                                            mut OCChannel: uint32_t) {
    /* Check the parameters */
    /* Disable the timer output */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.DISR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.DISR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OCChannel) as uint32_t
                                    as uint32_t);
    /* Disable the timer counter */
    if TimerIdxToTimerId[TimerIdx as usize] & 0x10000i32 as uint32_t ==
           0x10000i32 as uint32_t {
        ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x20000i32 as uint32_t ==
           0x20000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x2i32 as uint32_t | 0x1i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x40000i32 as uint32_t ==
           0x40000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x8i32 as uint32_t | 0x4i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x40000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x80000i32 as uint32_t ==
           0x80000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x20i32 as uint32_t | 0x10i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x80000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x100000i32 as uint32_t ==
           0x100000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x80i32 as uint32_t | 0x40i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x200000i32 as uint32_t ==
           0x200000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x200i32 as uint32_t | 0x100i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x200000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    };
}
/* *
  * @brief  Starts the PWM output signal generation on the designed timer output
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  PWMChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimplePWMStart(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut TimerIdx: uint32_t,
                                              mut PWMChannel: uint32_t) {
    /* Check the parameters */
    /* Enable the timer output */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.OENR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.OENR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | PWMChannel) as uint32_t
                                    as uint32_t);
    /* Enable the timer counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TimerIdxToTimerId[TimerIdx as usize]) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Stops the PWM output signal generation on the designed timer output
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  PWMChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimplePWMStop(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut PWMChannel: uint32_t) {
    /* Check the parameters */
    /* Disable the timer output */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.DISR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.DISR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | PWMChannel) as uint32_t
                                    as uint32_t);
    /* Disable the timer counter */
    if TimerIdxToTimerId[TimerIdx as usize] & 0x10000i32 as uint32_t ==
           0x10000i32 as uint32_t {
        ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x20000i32 as uint32_t ==
           0x20000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x2i32 as uint32_t | 0x1i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x40000i32 as uint32_t ==
           0x40000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x8i32 as uint32_t | 0x4i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x40000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x80000i32 as uint32_t ==
           0x80000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x20i32 as uint32_t | 0x10i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x80000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x100000i32 as uint32_t ==
           0x100000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x80i32 as uint32_t | 0x40i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x200000i32 as uint32_t ==
           0x200000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x200i32 as uint32_t | 0x100i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x200000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    };
}
/* *
  * @brief  Enables a basic capture on the designed capture unit
  * @param  HRTIMx: pointer to HRTIM peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureChannel: Timer output
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @retval None
  * @note  The external event triggering the capture is available for all timing 
  *        units. It can be used directly and is active as soon as the timing 
  *        unit counter is enabled.
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleCaptureStart(mut HRTIMx:
                                                      *mut HRTIM_TypeDef,
                                                  mut TimerIdx: uint32_t,
                                                  mut CaptureChannel:
                                                      uint32_t) {
    /* Enable the timer counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TimerIdxToTimerId[TimerIdx as usize]) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Disables a basic capture on the designed capture unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureChannel: Timer output
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleCaptureStop(mut HRTIMx:
                                                     *mut HRTIM_TypeDef,
                                                 mut TimerIdx: uint32_t,
                                                 mut CaptureChannel:
                                                     uint32_t) {
    /* Check the parameters */
    /* Set the capture unit trigger */
    match CaptureChannel {
        1 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT1xCR
                                            as *mut uint32_t,
                                        0i32 as uint32_t)
        }
        2 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT2xCR
                                            as *mut uint32_t,
                                        0i32 as uint32_t)
        }
        _ => { }
    }
    /* Disable the timer counter */
    if (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].CPT1xCR == 0i32 as uint32_t
           &&
           (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].CPT2xCR ==
               0i32 as uint32_t {
        if TimerIdxToTimerId[TimerIdx as usize] & 0x10000i32 as uint32_t ==
               0x10000i32 as uint32_t {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x10000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
        if TimerIdxToTimerId[TimerIdx as usize] & 0x20000i32 as uint32_t ==
               0x20000i32 as uint32_t {
            if (*HRTIMx).HRTIM_COMMON.OENR &
                   (0x2i32 as uint32_t | 0x1i32 as uint32_t) ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x20000i32 as uint32_t)) as
                                                uint32_t as uint32_t)
            }
        }
        if TimerIdxToTimerId[TimerIdx as usize] & 0x40000i32 as uint32_t ==
               0x40000i32 as uint32_t {
            if (*HRTIMx).HRTIM_COMMON.OENR &
                   (0x8i32 as uint32_t | 0x4i32 as uint32_t) ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x40000i32 as uint32_t)) as
                                                uint32_t as uint32_t)
            }
        }
        if TimerIdxToTimerId[TimerIdx as usize] & 0x80000i32 as uint32_t ==
               0x80000i32 as uint32_t {
            if (*HRTIMx).HRTIM_COMMON.OENR &
                   (0x20i32 as uint32_t | 0x10i32 as uint32_t) ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x80000i32 as uint32_t)) as
                                                uint32_t as uint32_t)
            }
        }
        if TimerIdxToTimerId[TimerIdx as usize] & 0x100000i32 as uint32_t ==
               0x100000i32 as uint32_t {
            if (*HRTIMx).HRTIM_COMMON.OENR &
                   (0x80i32 as uint32_t | 0x40i32 as uint32_t) ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x100000i32 as uint32_t))
                                                as uint32_t as uint32_t)
            }
        }
        if TimerIdxToTimerId[TimerIdx as usize] & 0x200000i32 as uint32_t ==
               0x200000i32 as uint32_t {
            if (*HRTIMx).HRTIM_COMMON.OENR &
                   (0x200i32 as uint32_t | 0x100i32 as uint32_t) ==
                   RESET as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                                *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint &
                                                 !(0x200000i32 as uint32_t))
                                                as uint32_t as uint32_t)
            }
        }
    };
}
/* *
  * @brief  Enables the basic one pulse signal generation on the designed output 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OnePulseChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOnePulseStart(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut TimerIdx: uint32_t,
                                                   mut OnePulseChannel:
                                                       uint32_t) {
    /* Check the parameters */
    /* Enable the timer output */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.OENR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.OENR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OnePulseChannel) as
                                    uint32_t as uint32_t);
    /* Enable the timer counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     TimerIdxToTimerId[TimerIdx as usize]) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Disables the basic one pulse signal generation on the designed output 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OnePulseChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOnePulseStop(mut HRTIMx:
                                                      *mut HRTIM_TypeDef,
                                                  mut TimerIdx: uint32_t,
                                                  mut OnePulseChannel:
                                                      uint32_t) {
    /* Check the parameters */
    /* Disable the timer output */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.DISR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.DISR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | OnePulseChannel) as
                                    uint32_t as uint32_t);
    /* Disable the timer counter */
    if TimerIdxToTimerId[TimerIdx as usize] & 0x10000i32 as uint32_t ==
           0x10000i32 as uint32_t {
        ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint &
                                         !(0x10000i32 as uint32_t)) as
                                        uint32_t as uint32_t)
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x20000i32 as uint32_t ==
           0x20000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x2i32 as uint32_t | 0x1i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x20000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x40000i32 as uint32_t ==
           0x40000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x8i32 as uint32_t | 0x4i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x40000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x80000i32 as uint32_t ==
           0x80000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x20i32 as uint32_t | 0x10i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x80000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x100000i32 as uint32_t ==
           0x100000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x80i32 as uint32_t | 0x40i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x100000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    }
    if TimerIdxToTimerId[TimerIdx as usize] & 0x200000i32 as uint32_t ==
           0x200000i32 as uint32_t {
        if (*HRTIMx).HRTIM_COMMON.OENR &
               (0x200i32 as uint32_t | 0x100i32 as uint32_t) ==
               RESET as libc::c_int as libc::c_uint {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint &
                                             !(0x200000i32 as uint32_t)) as
                                            uint32_t as uint32_t)
        }
    };
}
/* *
  * @brief  Starts the counter of the designated timer(s) operating in waveform mode
  *         Timers can be combined (ORed) to allow for simultaneous counter start
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimersToStart: Timer counter(s) to start
  *                   This parameter can be any combination of the following values:
  *                   @arg HRTIM_TIMERID_MASTER 
  *                   @arg HRTIM_TIMERID_TIMER_A 
  *                   @arg HRTIM_TIMERID_TIMER_B 
  *                   @arg HRTIM_TIMERID_TIMER_C 
  *                   @arg HRTIM_TIMERID_TIMER_D 
  *                   @arg HRTIM_TIMERID_TIMER_E 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformCounterStart(mut HRTIMx:
                                                        *mut HRTIM_TypeDef,
                                                    mut TimersToStart:
                                                        uint32_t) {
    /* Enable timer(s) counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | TimersToStart) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Stops the counter of the designated timer(s) operating in waveform mode
  *         Timers can be combined (ORed) to allow for simultaneous counter stop
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimersToStop: Timer counter(s) to stop
  *                   This parameter can be any combination of the following values:
  *                   @arg HRTIM_TIMERID_MASTER 
  *                   @arg HRTIM_TIMERID_TIMER_A 
  *                   @arg HRTIM_TIMERID_TIMER_B 
  *                   @arg HRTIM_TIMERID_TIMER_C 
  *                   @arg HRTIM_TIMERID_TIMER_D 
  *                   @arg HRTIM_TIMERID_TIMER_E 
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformCounterStop(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut TimersToStop:
                                                       uint32_t) {
    /* Disable timer(s) counter */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !TimersToStop) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Enables the generation of the waveform signal on the designated output(s)
  *         Outputs can be combined (ORed) to allow for simultaneous output enabling
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  OutputsToStart: Timer output(s) to enable
  *                    This parameter can be any combination of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformOutputStart(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut OutputsToStart:
                                                       uint32_t) {
    /* Enable the HRTIM outputs */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.OENR as
                                    *mut uint32_t, OutputsToStart);
}
/* *
  * @brief  Disables the generation of the waveform signal on the designated output(s)
  *         Outputs can be combined (ORed) to allow for simultaneous output disabling
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  OutputsToStop: Timer output(s) to disable
  *                    This parameter can be any combination of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformOutputStop(mut HRTIMx:
                                                      *mut HRTIM_TypeDef,
                                                  mut OutputsToStop:
                                                      uint32_t) {
    /* Disable the HRTIM outputs */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.DISR as
                                    *mut uint32_t, OutputsToStop);
}
/* *
  * @brief  Enables or disables the Master and slaves interrupt request
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  HRTIM_IT: specifies the HRTIM interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_MASTER_IT_MCMP1: Master compare 1 interrupt source
  *            @arg HRTIM_MASTER_IT_MCMP2: Master compare 2 interrupt source
  *            @arg HRTIM_MASTER_IT_MCMP3: Master compare 3 interrupt Interrupt source
  *            @arg HRTIM_MASTER_IT_MCMP4: Master compare 4 Interrupt source
  *            @arg HRTIM_MASTER_IT_MREP: Master Repetition Interrupt source
  *            @arg HRTIM_MASTER_IT_SYNC: Synchronization input Interrupt source
  *            @arg HRTIM_MASTER_IT_MUPD: Master update Interrupt source
  *            @arg HRTIM_TIM_IT_CMP1: Timer compare 1 Interrupt source
  *            @arg HRTIM_TIM_IT_CMP2: Timer compare 2 Interrupt source
  *            @arg HRTIM_TIM_IT_CMP3: Timer compare 3 Interrupt source
  *            @arg HRTIM_TIM_IT_CMP4: Timer compare 4 Interrupt source
  *            @arg HRTIM_TIM_IT_REP: Timer repetition Interrupt source
  *            @arg HRTIM_TIM_IT_UPD: Timer update Interrupt source
  *            @arg HRTIM_TIM_IT_CPT1: Timer capture 1 Interrupt source
  *            @arg HRTIM_TIM_IT_CPT2: Timer capture 2 Interrupt source
  *            @arg HRTIM_TIM_IT_SET1: Timer output 1 set Interrupt source
  *            @arg HRTIM_TIM_IT_RST1: Timer output 1 reset Interrupt source
  *            @arg HRTIM_TIM_IT_SET2: Timer output 2 set Interrupt source
  *            @arg HRTIM_TIM_IT_RST2: Timer output 2 reset Interrupt source
  *            @arg HRTIM_TIM_IT_RST: Timer reset Interrupt source
  *            @arg HRTIM_TIM_IT_DLYPRT1: Timer delay protection Interrupt source
  * @param  NewState: new state of the TIM interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ITConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                        mut TimerIdx: uint32_t,
                                        mut HRTIM_IT: uint32_t,
                                        mut NewState: FunctionalState) {
    match TimerIdx {
        5 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint | HRTIM_IT)
                                                as uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !HRTIM_IT)
                                                as uint32_t as uint32_t)
            }
        }
        0 | 1 | 2 | 3 | 4 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].TIMxDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].TIMxDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint | HRTIM_IT)
                                                as uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].TIMxDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].TIMxDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !HRTIM_IT)
                                                as uint32_t as uint32_t)
            }
        }
        _ => { }
    };
}
/* *
  * @brief  Enables or disables the common interrupt request
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  HRTIM_IT: specifies the HRTIM interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_IT_FLT1: Fault 1 interrupt source
  *            @arg HRTIM_IT_FLT2: Fault 2 interrupt source
  *            @arg HRTIM_IT_FLT3: Fault 3 interrupt Interrupt source
  *            @arg HRTIM_IT_FLT4: Fault 4 Interrupt source
  *            @arg HRTIM_IT_FLT5: Fault 5  Interrupt source
  *            @arg HRTIM_IT_SYSFLT: System Fault Interrupt source
  *            @arg HRTIM_IT_DLLRDY: DLL ready Interrupt source
  *            @arg HRTIM_IT_BMPER: Burst mode period Interrupt source
  * @param  NewState: new state of the TIM interrupts.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ITCommonConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut HRTIM_CommonIT: uint32_t,
                                              mut NewState: FunctionalState) {
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.IER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.IER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | HRTIM_CommonIT) as
                                        uint32_t as uint32_t)
    } else {
        ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.IER as
                                        *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.IER
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !HRTIM_CommonIT) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Clears the Master and slaves interrupt flags
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  HRTIM_FLAG: specifies the HRTIM flags sources to be cleared.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_MASTER_FLAG_MCMP1: Master compare 1 interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MCMP2: Master compare 2 interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MCMP3: Master compare 3 interrupt Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MCMP4: Master compare 4 Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MREP: Master Repetition Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_SYNC: Synchronization input Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MUPD: Master update Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP1: Timer compare 1 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP2: Timer compare 2 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP3: Timer compare 3 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP4: Timer compare 4 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_REP: Timer repetition Interrupt flag
  *            @arg HRTIM_TIM_FLAG_UPD: Timer update Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CPT1: Timer capture 1 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CPT2: Timer capture 2 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_SET1: Timer output 1 set Interrupt flag
  *            @arg HRTIM_TIM_FLAG_RST1: Timer output 1 reset Interrupt flag
  *            @arg HRTIM_TIM_FLAG_SET2: Timer output 2 set Interrupt flag
  *            @arg HRTIM_TIM_FLAG_RST2: Timer output 2 reset Interrupt flag
  *            @arg HRTIM_TIM_FLAG_RST: Timer reset Interrupt flag
  *            @arg HRTIM_TIM_FLAG_DLYPRT1: Timer delay protection Interrupt flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ClearFlag(mut HRTIMx: *mut HRTIM_TypeDef,
                                         mut TimerIdx: uint32_t,
                                         mut HRTIM_FLAG: uint32_t) {
    match TimerIdx {
        5 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MICR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MICR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | HRTIM_FLAG) as
                                            uint32_t as uint32_t)
        }
        0 | 1 | 2 | 3 | 4 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].TIMxICR
                                            as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                            as
                                                                                                            usize].TIMxICR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | HRTIM_FLAG) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    };
}
/* *
  * @brief  Clears the common interrupt flags
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  HRTIM_FLAG: specifies the HRTIM flags to be cleared.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_FLAG_FLT1: Fault 1 interrupt flag
  *            @arg HRTIM_FLAG_FLT2: Fault 2 interrupt flag
  *            @arg HRTIM_FLAG_FLT3: Fault 3 interrupt Interrupt flag
  *            @arg HRTIM_FLAG_FLT4: Fault 4 Interrupt flag
  *            @arg HRTIM_FLAG_FLT5: Fault 5  Interrupt flag
  *            @arg HRTIM_FLAG_SYSFLT: System Fault Interrupt flag
  *            @arg HRTIM_FLAG_DLLRDY: DLL ready Interrupt flag
  *            @arg HRTIM_FLAG_BMPER: Burst mode period Interrupt flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ClearCommonFlag(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut HRTIM_CommonFLAG:
                                                   uint32_t) {
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.ICR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.ICR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | HRTIM_CommonFLAG) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Clears the Master and slaves interrupt request pending bits
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  HRTIM_IT: specifies the HRTIM interrupts sources to be enabled or disabled.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_MASTER_IT_MCMP1: Master compare 1 interrupt source
  *            @arg HRTIM_MASTER_IT_MCMP2: Master compare 2 interrupt source
  *            @arg HRTIM_MASTER_IT_MCMP3: Master compare 3 interrupt Interrupt source
  *            @arg HRTIM_MASTER_IT_MCMP4: Master compare 4 Interrupt source
  *            @arg HRTIM_MASTER_IT_MREP: Master Repetition Interrupt source
  *            @arg HRTIM_MASTER_IT_SYNC: Synchronization input Interrupt source
  *            @arg HRTIM_MASTER_IT_MUPD: Master update Interrupt source
  *            @arg HRTIM_TIM_IT_CMP1: Timer compare 1 Interrupt source
  *            @arg HRTIM_TIM_IT_CMP2: Timer compare 2 Interrupt source
  *            @arg HRTIM_TIM_IT_CMP3: Timer compare 3 Interrupt source
  *            @arg HRTIM_TIM_IT_CMP4: Timer compare 4 Interrupt source
  *            @arg HRTIM_TIM_IT_REP: Timer repetition Interrupt source
  *            @arg HRTIM_TIM_IT_UPD: Timer update Interrupt source
  *            @arg HRTIM_TIM_IT_CPT1: Timer capture 1 Interrupt source
  *            @arg HRTIM_TIM_IT_CPT2: Timer capture 2 Interrupt source
  *            @arg HRTIM_TIM_IT_SET1: Timer output 1 set Interrupt source
  *            @arg HRTIM_TIM_IT_RST1: Timer output 1 reset Interrupt source
  *            @arg HRTIM_TIM_IT_SET2: Timer output 2 set Interrupt source
  *            @arg HRTIM_TIM_IT_RST2: Timer output 2 reset Interrupt source
  *            @arg HRTIM_TIM_IT_RST: Timer reset Interrupt source
  *            @arg HRTIM_TIM_IT_DLYPRT: Timer delay protection Interrupt source
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ClearITPendingBit(mut HRTIMx:
                                                     *mut HRTIM_TypeDef,
                                                 mut TimerIdx: uint32_t,
                                                 mut HRTIM_IT: uint32_t) {
    match TimerIdx {
        5 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MICR as
                                            *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MICR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | HRTIM_IT) as
                                            uint32_t as uint32_t)
        }
        0 | 1 | 2 | 3 | 4 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].TIMxICR
                                            as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                            as
                                                                                                            usize].TIMxICR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint | HRTIM_IT) as
                                            uint32_t as uint32_t)
        }
        _ => { }
    };
}
/* *
  * @brief  Clears the common interrupt pending bits
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  HRTIM_IT: specifies the HRTIM interrupts sources to be cleared.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_IT_FLT1: Fault 1 interrupt source
  *            @arg HRTIM_IT_FLT2: Fault 2 interrupt source
  *            @arg HRTIM_IT_FLT3: Fault 3 interrupt Interrupt source
  *            @arg HRTIM_IT_FLT4: Fault 4 Interrupt source
  *            @arg HRTIM_IT_FLT5: Fault 5  Interrupt source
  *            @arg HRTIM_IT_SYSFLT: System Fault Interrupt source
  *            @arg HRTIM_IT_DLLRDY: DLL ready Interrupt source
  *            @arg HRTIM_IT_BMPER: Burst mode period Interrupt source
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ClearCommonITPendingBit(mut HRTIMx:
                                                           *mut HRTIM_TypeDef,
                                                       mut HRTIM_CommonIT:
                                                           uint32_t) {
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.ICR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.ICR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | HRTIM_CommonIT) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Checks whether the specified HRTIM flag is set or not.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  HRTIM_FLAG: specifies the HRTIM flags to check.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_MASTER_FLAG_MCMP1: Master compare 1 interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MCMP2: Master compare 2 interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MCMP3: Master compare 3 interrupt Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MCMP4: Master compare 4 Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MREP: Master Repetition Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_SYNC: Synchronization input Interrupt flag
  *            @arg HRTIM_MASTER_FLAG_MUPD: Master update Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP1: Timer compare 1 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP2: Timer compare 2 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP3: Timer compare 3 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CMP4: Timer compare 4 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_REP: Timer repetition Interrupt flag
  *            @arg HRTIM_TIM_FLAG_UPD: Timer update Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CPT1: Timer capture 1 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_CPT2: Timer capture 2 Interrupt flag
  *            @arg HRTIM_TIM_FLAG_SET1: Timer output 1 set Interrupt flag
  *            @arg HRTIM_TIM_FLAG_RST1: Timer output 1 reset Interrupt flag
  *            @arg HRTIM_TIM_FLAG_SET2: Timer output 2 set Interrupt flag
  *            @arg HRTIM_TIM_FLAG_RST2: Timer output 2 reset Interrupt flag
  *            @arg HRTIM_TIM_FLAG_RST: Timer reset Interrupt flag
  *            @arg HRTIM_TIM_FLAG_DLYPRT: Timer delay protection Interrupt flag
  * @retval The new state of HRTIM_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetFlagStatus(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut HRTIM_FLAG: uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    match TimerIdx {
        5 => {
            if (*HRTIMx).HRTIM_MASTER.MISR & HRTIM_FLAG !=
                   RESET as libc::c_int as libc::c_uint {
                bitstatus = SET
            } else { bitstatus = RESET }
        }
        0 | 1 | 2 | 3 | 4 => {
            if (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR & HRTIM_FLAG
                   != RESET as libc::c_int as libc::c_uint {
                bitstatus = SET
            } else { bitstatus = RESET }
        }
        _ => { }
    }
    return bitstatus;
}
/* *
  * @brief  Checks whether the specified HRTIM common flag is set or not.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  HRTIM_FLAG: specifies the HRTIM flags to check.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_FLAG_FLT1: Fault 1 interrupt flag
  *            @arg HRTIM_FLAG_FLT2: Fault 2 interrupt flag
  *            @arg HRTIM_FLAG_FLT3: Fault 3 interrupt Interrupt flag
  *            @arg HRTIM_FLAG_FLT4: Fault 4 Interrupt flag
  *            @arg HRTIM_FLAG_FLT5: Fault 5  Interrupt flag
  *            @arg HRTIM_FLAG_SYSFLT: System Fault Interrupt flag
  *            @arg HRTIM_FLAG_DLLRDY: DLL ready Interrupt flag
  *            @arg HRTIM_FLAG_BMPER: Burst mode period Interrupt flag
  * @retval The new state of HRTIM_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetCommonFlagStatus(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut HRTIM_CommonFLAG:
                                                       uint32_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    if (*HRTIMx).HRTIM_COMMON.ISR & HRTIM_CommonFLAG !=
           RESET as libc::c_int as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Checks whether the specified HRTIM interrupt has occurred or not.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  HRTIM_IT: specifies the HRTIM flags sources to be cleared.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_MASTER_IT_MCMP1: Master compare 1 interrupt 
  *            @arg HRTIM_MASTER_IT_MCMP2: Master compare 2 interrupt 
  *            @arg HRTIM_MASTER_IT_MCMP3: Master compare 3 interrupt Interrupt 
  *            @arg HRTIM_MASTER_IT_MCMP4: Master compare 4 Interrupt 
  *            @arg HRTIM_MASTER_IT_MREP: Master Repetition Interrupt 
  *            @arg HRTIM_MASTER_IT_SYNC: Synchronization input Interrupt 
  *            @arg HRTIM_MASTER_IT_MUPD: Master update Interrupt 
  *            @arg HRTIM_TIM_IT_CMP1: Timer compare 1 Interrupt 
  *            @arg HRTIM_TIM_IT_CMP2: Timer compare 2 Interrupt 
  *            @arg HRTIM_TIM_IT_CMP3: Timer compare 3 Interrupt 
  *            @arg HRTIM_TIM_IT_CMP4: Timer compare 4 Interrupt 
  *            @arg HRTIM_TIM_IT_REP: Timer repetition Interrupt 
  *            @arg HRTIM_TIM_IT_UPD: Timer update Interrupt 
  *            @arg HRTIM_TIM_IT_CPT1: Timer capture 1 Interrupt 
  *            @arg HRTIM_TIM_IT_CPT2: Timer capture 2 Interrupt 
  *            @arg HRTIM_TIM_IT_SET1: Timer output 1 set Interrupt 
  *            @arg HRTIM_TIM_IT_RST1: Timer output 1 reset Interrupt 
  *            @arg HRTIM_TIM_IT_SET2: Timer output 2 set Interrupt 
  *            @arg HRTIM_TIM_IT_RST2: Timer output 2 reset Interrupt 
  *            @arg HRTIM_TIM_IT_RST: Timer reset Interrupt 
  *            @arg HRTIM_TIM_IT_DLYPRT: Timer delay protection Interrupt 
  * @retval The new state of the HRTIM_IT(SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetITStatus(mut HRTIMx: *mut HRTIM_TypeDef,
                                           mut TimerIdx: uint32_t,
                                           mut HRTIM_IT: uint32_t)
 -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itstatus: uint16_t = 0i32 as uint16_t;
    let mut itenable: uint16_t = 0i32 as uint16_t;
    match TimerIdx {
        5 => {
            itstatus = ((*HRTIMx).HRTIM_MASTER.MISR & HRTIM_IT) as uint16_t;
            itenable = ((*HRTIMx).HRTIM_MASTER.MDIER & HRTIM_IT) as uint16_t;
            if itstatus as libc::c_int !=
                   RESET as libc::c_int as uint16_t as libc::c_int &&
                   itenable as libc::c_int !=
                       RESET as libc::c_int as uint16_t as libc::c_int {
                bitstatus = SET
            } else { bitstatus = RESET }
        }
        0 | 1 | 2 | 3 | 4 => {
            itstatus =
                ((*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR & HRTIM_IT)
                    as uint16_t;
            itenable =
                ((*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxDIER &
                     HRTIM_IT) as uint16_t;
            if itstatus as libc::c_int !=
                   RESET as libc::c_int as uint16_t as libc::c_int &&
                   itenable as libc::c_int !=
                       RESET as libc::c_int as uint16_t as libc::c_int {
                bitstatus = SET
            } else { bitstatus = RESET }
        }
        _ => { }
    }
    return bitstatus;
}
/* *
  * @brief  Checks whether the specified HRTIM common interrupt has occurred or not.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  HRTIM_IT: specifies the HRTIM interrupt source to check.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_IT_FLT1: Fault 1 interrupt 
  *            @arg HRTIM_IT_FLT2: Fault 2 interrupt 
  *            @arg HRTIM_IT_FLT3: Fault 3 interrupt Interrupt 
  *            @arg HRTIM_IT_FLT4: Fault 4 Interrupt 
  *            @arg HRTIM_IT_FLT5: Fault 5  Interrupt 
  *            @arg HRTIM_IT_SYSFLT: System Fault Interrupt 
  *            @arg HRTIM_IT_DLLRDY: DLL ready Interrupt flag
  *            @arg HRTIM_IT_BMPER: Burst mode period Interrupt 
  * @retval The new state of HRTIM_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetCommonITStatus(mut HRTIMx:
                                                     *mut HRTIM_TypeDef,
                                                 mut HRTIM_CommonIT: uint32_t)
 -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    let mut itstatus: uint16_t = 0i32 as uint16_t;
    let mut itenable: uint16_t = 0i32 as uint16_t;
    itstatus = ((*HRTIMx).HRTIM_COMMON.ISR & HRTIM_CommonIT) as uint16_t;
    itenable = ((*HRTIMx).HRTIM_COMMON.IER & HRTIM_CommonIT) as uint16_t;
    if itstatus as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int &&
           itenable as libc::c_int !=
               RESET as libc::c_int as uint16_t as libc::c_int {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Enables or disables the HRTIMx's DMA Requests.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  HRTIM_DMA: specifies the DMA Request sources.
  *          This parameter can be any combination of the following values:
  *            @arg HRTIM_MASTER_DMA_MCMP1: Master compare 1 DMA request source
  *            @arg HRTIM_MASTER_DMA_MCMP2: Master compare 2 DMA request source
  *            @arg HRTIM_MASTER_DMA_MCMP3: Master compare 3 DMA request source
  *            @arg HRTIM_MASTER_DMA_MCMP4: Master compare 4 DMA request source
  *            @arg HRTIM_MASTER_DMA_MREP: Master Repetition DMA request source
  *            @arg HRTIM_MASTER_DMA_SYNC: Synchronization input DMA request source
  *            @arg HRTIM_MASTER_DMA_MUPD:Master update DMA request source
  *            @arg HRTIM_TIM_DMA_CMP1: Timer compare 1 DMA request source 
  *            @arg HRTIM_TIM_DMA_CMP2: Timer compare 2 DMA request source 
  *            @arg HRTIM_TIM_DMA_CMP3: Timer compare 3 DMA request source 
  *            @arg HRTIM_TIM_DMA_CMP4: Timer compare 4 DMA request source 
  *            @arg HRTIM_TIM_DMA_REP: Timer repetition DMA request source 
  *            @arg HRTIM_TIM_DMA_UPD: Timer update DMA request source 
  *            @arg HRTIM_TIM_DMA_CPT1: Timer capture 1 DMA request source 
  *            @arg HRTIM_TIM_DMA_CPT2: Timer capture 2 DMA request source 
  *            @arg HRTIM_TIM_DMA_SET1: Timer output 1 set DMA request source 
  *            @arg HRTIM_TIM_DMA_RST1: Timer output 1 reset DMA request source 
  *            @arg HRTIM_TIM_DMA_SET2: Timer output 2 set DMA request source 
  *            @arg HRTIM_TIM_DMA_RST2: Timer output 2 reset DMA request source 
  *            @arg HRTIM_TIM_DMA_RST: Timer reset DMA request source 
  *            @arg HRTIM_TIM_DMA_DLYPRT: Timer delay protection DMA request source 
  * @param  NewState: new state of the DMA Request sources.
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_DMACmd(mut HRTIMx: *mut HRTIM_TypeDef,
                                      mut TimerIdx: uint32_t,
                                      mut HRTIM_DMA: uint32_t,
                                      mut NewState: FunctionalState) {
    match TimerIdx {
        5 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint | HRTIM_DMA)
                                                as uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !HRTIM_DMA)
                                                as uint32_t as uint32_t)
            }
        }
        0 | 1 | 2 | 3 | 4 => {
            if NewState as libc::c_uint !=
                   DISABLE as libc::c_int as libc::c_uint {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].TIMxDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].TIMxDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint | HRTIM_DMA)
                                                as uint32_t as uint32_t)
            } else {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].TIMxDIER
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].TIMxDIER
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint & !HRTIM_DMA)
                                                as uint32_t as uint32_t)
            }
        }
        _ => { }
    };
}
/* *
  * @}
  */
/* * @defgroup HRTIM_Group3 Peripheral Control methods 
 *  @brief   management functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral Control methods #####
 ===============================================================================  
    [..]
    This subsection provides a set of functions allowing to control the HRTIMx data 
    transfers.

@endverbatim
  * @{
  */
/* *
  * @brief  Configures an output in basic output compare mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OCChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pBasicOCChannelCfg: pointer to the basic output compare output configuration structure
  * @note When the timer operates in basic output compare mode:
  *         Output 1 is implicitely controled by the compare unit 1
  *         Output 2 is implicitely controled by the compare unit 2
  *       Output Set/Reset crossbar is set according to the selected output compare mode:
  *         Toggle: SETxyR = RSTxyR = CMPy
  *         Active: SETxyR = CMPy, RSTxyR = 0
  *         Inactive: SETxy =0, RSTxy = CMPy
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOCChannelConfig(mut HRTIMx:
                                                         *mut HRTIM_TypeDef,
                                                     mut TimerIdx: uint32_t,
                                                     mut OCChannel: uint32_t,
                                                     mut pBasicOCChannelCfg:
                                                         *mut HRTIM_BasicOCChannelCfgTypeDef) {
    let mut CompareUnit: uint32_t = 0x1i32 as uint32_t;
    let mut CompareCfg: HRTIM_CompareCfgTypeDef =
        HRTIM_CompareCfgTypeDef{CompareValue: 0,
                                AutoDelayedMode: 0,
                                AutoDelayedTimeout: 0,};
    let mut OutputCfg: HRTIM_OutputCfgTypeDef =
        HRTIM_OutputCfgTypeDef{Polarity: 0,
                               SetSource: 0,
                               ResetSource: 0,
                               IdleMode: 0,
                               IdleState: 0,
                               FaultState: 0,
                               ChopperModeEnable: 0,
                               BurstModeEntryDelayed: 0,};
    /* Check parameters */
    /* Configure timer compare unit */
    match OCChannel {
        1 | 4 | 16 | 64 | 256 => { CompareUnit = 0x1i32 as uint32_t }
        2 | 8 | 32 | 128 | 512 => { CompareUnit = 0x2i32 as uint32_t }
        _ => { }
    }
    CompareCfg.CompareValue = (*pBasicOCChannelCfg).Pulse;
    CompareCfg.AutoDelayedMode = 0i32 as uint32_t;
    CompareCfg.AutoDelayedTimeout = 0i32 as uint32_t;
    HRTIM_CompareUnitConfig(HRTIMx, TimerIdx, CompareUnit, &mut CompareCfg);
    /* Configure timer output */
    OutputCfg.Polarity = (*pBasicOCChannelCfg).Polarity;
    OutputCfg.IdleState = (*pBasicOCChannelCfg).IdleState;
    OutputCfg.FaultState = 0i32 as uint32_t;
    OutputCfg.IdleMode = 0i32 as uint32_t;
    OutputCfg.ChopperModeEnable = 0i32 as uint32_t;
    OutputCfg.BurstModeEntryDelayed = 0i32 as uint32_t;
    match (*pBasicOCChannelCfg).Mode {
        1 => {
            if CompareUnit == 0x1i32 as uint32_t {
                OutputCfg.SetSource = 0x8i32 as uint32_t
            } else { OutputCfg.SetSource = 0x10i32 as uint32_t }
            OutputCfg.ResetSource = OutputCfg.SetSource
        }
        3 => {
            if CompareUnit == 0x1i32 as uint32_t {
                OutputCfg.SetSource = 0x8i32 as uint32_t
            } else { OutputCfg.SetSource = 0x10i32 as uint32_t }
            OutputCfg.ResetSource = 0i32 as uint32_t
        }
        2 => {
            if CompareUnit == 0x1i32 as uint32_t {
                OutputCfg.ResetSource = 0x8i32 as uint32_t
            } else { OutputCfg.ResetSource = 0x10i32 as uint32_t }
            OutputCfg.SetSource = 0i32 as uint32_t
        }
        _ => { }
    }
    HRTIM_OutputConfig(HRTIMx, TimerIdx, OCChannel, &mut OutputCfg);
}
/* *
  * @brief  Configures an output in basic PWM mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  PWMChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pBasicPWMChannelCfg: pointer to the basic PWM output configuration structure
  * @note When the timer operates in basic PWM output mode:
  *         Output 1 is implicitly controled by the compare unit 1
  *         Output 2 is implicitly controled by the compare unit 2
  *         Output Set/Reset crossbar is set as follows:
  *         Output 1: SETx1R = CMP1, RSTx1R = PER
  *         Output 2: SETx2R = CMP2, RST2R = PER
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimplePWMChannelConfig(mut HRTIMx:
                                                          *mut HRTIM_TypeDef,
                                                      mut TimerIdx: uint32_t,
                                                      mut PWMChannel:
                                                          uint32_t,
                                                      mut pBasicPWMChannelCfg:
                                                          *mut HRTIM_BasicPWMChannelCfgTypeDef) {
    let mut CompareUnit: uint32_t = 0x1i32 as uint32_t;
    let mut CompareCfg: HRTIM_CompareCfgTypeDef =
        HRTIM_CompareCfgTypeDef{CompareValue: 0,
                                AutoDelayedMode: 0,
                                AutoDelayedTimeout: 0,};
    let mut OutputCfg: HRTIM_OutputCfgTypeDef =
        HRTIM_OutputCfgTypeDef{Polarity: 0,
                               SetSource: 0,
                               ResetSource: 0,
                               IdleMode: 0,
                               IdleState: 0,
                               FaultState: 0,
                               ChopperModeEnable: 0,
                               BurstModeEntryDelayed: 0,};
    /* Check parameters */
    /* Configure timer compare unit */
    match PWMChannel {
        1 | 4 | 16 | 64 | 256 => { CompareUnit = 0x1i32 as uint32_t }
        2 | 8 | 32 | 128 | 512 => { CompareUnit = 0x2i32 as uint32_t }
        _ => { }
    }
    CompareCfg.CompareValue = (*pBasicPWMChannelCfg).Pulse;
    CompareCfg.AutoDelayedMode = 0i32 as uint32_t;
    CompareCfg.AutoDelayedTimeout = 0i32 as uint32_t;
    HRTIM_CompareUnitConfig(HRTIMx, TimerIdx, CompareUnit, &mut CompareCfg);
    /* Configure timer output */
    OutputCfg.Polarity = (*pBasicPWMChannelCfg).Polarity;
    OutputCfg.IdleState = (*pBasicPWMChannelCfg).IdleState;
    OutputCfg.FaultState = 0i32 as uint32_t;
    OutputCfg.IdleMode = 0i32 as uint32_t;
    OutputCfg.ChopperModeEnable = 0i32 as uint32_t;
    OutputCfg.BurstModeEntryDelayed = 0i32 as uint32_t;
    if CompareUnit == 0x1i32 as uint32_t {
        OutputCfg.SetSource = 0x8i32 as uint32_t
    } else { OutputCfg.SetSource = 0x10i32 as uint32_t }
    OutputCfg.ResetSource = 0x4i32 as uint32_t;
    HRTIM_OutputConfig(HRTIMx, TimerIdx, PWMChannel, &mut OutputCfg);
}
/* *
  * @brief  Configures a basic capture 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureChannel: Capture unit
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @param  pBasicCaptureChannelCfg: pointer to the basic capture configuration structure
  * @note When the timer operates in basic capture mode the capture is triggered
  *       by the designated external event and GPIO input is implicitly used as event source.
  *       The cature can be triggered by a rising edge, a falling edge or both
  *       edges on event channel.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleCaptureChannelConfig(mut HRTIMx:
                                                              *mut HRTIM_TypeDef,
                                                          mut TimerIdx:
                                                              uint32_t,
                                                          mut CaptureChannel:
                                                              uint32_t,
                                                          mut pBasicCaptureChannelCfg:
                                                              *mut HRTIM_BasicCaptureChannelCfgTypeDef) {
    let mut EventCfg: HRTIM_EventCfgTypeDef =
        HRTIM_EventCfgTypeDef{Source: 0,
                              Polarity: 0,
                              Sensitivity: 0,
                              Filter: 0,
                              FastMode: 0,};
    /* Check parameters */
    /* Configure external event channel */
    EventCfg.FastMode = 0i32 as uint32_t;
    EventCfg.Filter = (*pBasicCaptureChannelCfg).EventFilter;
    EventCfg.Polarity = (*pBasicCaptureChannelCfg).EventPolarity;
    EventCfg.Sensitivity = (*pBasicCaptureChannelCfg).EventSensitivity;
    EventCfg.Source = 0i32 as uint32_t;
    HRTIM_ExternalEventConfig(HRTIMx, (*pBasicCaptureChannelCfg).Event,
                              &mut EventCfg);
    /* Memorize capture trigger (will be configured when the capture is started */
    HRTIM_CaptureUnitConfig(HRTIMx, TimerIdx, CaptureChannel,
                            (*pBasicCaptureChannelCfg).Event);
}
/* *
  * @brief  Configures an output basic one pulse mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  OnePulseChannel: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pBasicOnePulseChannelCfg: pointer to the basic one pulse output configuration structure
  * @note When the timer operates in basic one pulse mode:
  *         the timer counter is implicitly started by the reset event,
  *         the reset of the timer counter is triggered by the designated external event
  *         GPIO input is implicitly used as event source,
  *         Output 1 is implicitly controled by the compare unit 1,
  *         Output 2 is implicitly controled by the compare unit 2.
  *         Output Set/Reset crossbar is set as follows:
  *         Output 1: SETx1R = CMP1, RSTx1R = PER
  *         Output 2: SETx2R = CMP2, RST2R = PER
  *         The counter mode should be HRTIM_MODE_SINGLESHOT_RETRIGGERABLE
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SimpleOnePulseChannelConfig(mut HRTIMx:
                                                               *mut HRTIM_TypeDef,
                                                           mut TimerIdx:
                                                               uint32_t,
                                                           mut OnePulseChannel:
                                                               uint32_t,
                                                           mut pBasicOnePulseChannelCfg:
                                                               *mut HRTIM_BasicOnePulseChannelCfgTypeDef) {
    let mut CompareUnit: uint32_t = 0x1i32 as uint32_t;
    let mut CompareCfg: HRTIM_CompareCfgTypeDef =
        HRTIM_CompareCfgTypeDef{CompareValue: 0,
                                AutoDelayedMode: 0,
                                AutoDelayedTimeout: 0,};
    let mut OutputCfg: HRTIM_OutputCfgTypeDef =
        HRTIM_OutputCfgTypeDef{Polarity: 0,
                               SetSource: 0,
                               ResetSource: 0,
                               IdleMode: 0,
                               IdleState: 0,
                               FaultState: 0,
                               ChopperModeEnable: 0,
                               BurstModeEntryDelayed: 0,};
    let mut EventCfg: HRTIM_EventCfgTypeDef =
        HRTIM_EventCfgTypeDef{Source: 0,
                              Polarity: 0,
                              Sensitivity: 0,
                              Filter: 0,
                              FastMode: 0,};
    /* Check parameters */
    /* Configure timer compare unit */
    match OnePulseChannel {
        1 | 4 | 16 | 64 | 256 => { CompareUnit = 0x1i32 as uint32_t }
        2 | 8 | 32 | 128 | 512 => { CompareUnit = 0x2i32 as uint32_t }
        _ => { }
    }
    CompareCfg.CompareValue = (*pBasicOnePulseChannelCfg).Pulse;
    CompareCfg.AutoDelayedMode = 0i32 as uint32_t;
    CompareCfg.AutoDelayedTimeout = 0i32 as uint32_t;
    HRTIM_CompareUnitConfig(HRTIMx, TimerIdx, CompareUnit, &mut CompareCfg);
    /* Configure timer output */
    OutputCfg.Polarity = (*pBasicOnePulseChannelCfg).OutputPolarity;
    OutputCfg.IdleState = (*pBasicOnePulseChannelCfg).OutputIdleState;
    OutputCfg.FaultState = 0i32 as uint32_t;
    OutputCfg.IdleMode = 0i32 as uint32_t;
    OutputCfg.ChopperModeEnable = 0i32 as uint32_t;
    OutputCfg.BurstModeEntryDelayed = 0i32 as uint32_t;
    if CompareUnit == 0x1i32 as uint32_t {
        OutputCfg.SetSource = 0x8i32 as uint32_t
    } else { OutputCfg.SetSource = 0x10i32 as uint32_t }
    OutputCfg.ResetSource = 0x4i32 as uint32_t;
    HRTIM_OutputConfig(HRTIMx, TimerIdx, OnePulseChannel, &mut OutputCfg);
    /* Configure external event channel */
    EventCfg.FastMode = 0i32 as uint32_t;
    EventCfg.Filter = (*pBasicOnePulseChannelCfg).EventFilter;
    EventCfg.Polarity = (*pBasicOnePulseChannelCfg).EventPolarity;
    EventCfg.Sensitivity = (*pBasicOnePulseChannelCfg).EventSensitivity;
    EventCfg.Source = 0i32 as uint32_t;
    HRTIM_ExternalEventConfig(HRTIMx, (*pBasicOnePulseChannelCfg).Event,
                              &mut EventCfg);
    /* Configure the timer reset register */
    HRTIM_TIM_ResetConfig(HRTIMx, TimerIdx,
                          (*pBasicOnePulseChannelCfg).Event);
}
/* *
  * @brief  Configures the general behavior of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  pTimerCfg: pointer to the timer configuration structure
  * @note When the timer operates in waveform mode, all the features supported by
  *       the HRTIMx are available without any limitation.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformTimerConfig(mut HRTIMx:
                                                       *mut HRTIM_TypeDef,
                                                   mut TimerIdx: uint32_t,
                                                   mut pTimerCfg:
                                                       *mut HRTIM_TimerCfgTypeDef) {
    let mut HRTIM_timcr: uint32_t = 0;
    let mut HRTIM_timfltr: uint32_t = 0;
    let mut HRTIM_timoutr: uint32_t = 0;
    let mut HRTIM_timrstr: uint32_t = 0;
    /* Check parameters */
    /* Configure timing unit (Timer A to Timer E) */
    HRTIM_timcr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxCR;
    HRTIM_timfltr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].FLTxR;
    HRTIM_timoutr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].OUTxR;
    HRTIM_timrstr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].RSTxR;
    /* Set the push-pull mode */
    HRTIM_timcr &= !(0x40i32 as uint32_t);
    HRTIM_timcr |= (*pTimerCfg).PushPull;
    /* Enable/Disable registers update on timer counter reset */
    HRTIM_timcr &= !(0x40000i32 as uint32_t);
    HRTIM_timcr |= (*pTimerCfg).ResetUpdate;
    /* Set the timer update trigger */
    HRTIM_timcr &=
        !(0x1000000i32 as uint32_t | 0x80000i32 as uint32_t |
              0x100000i32 as uint32_t | 0x200000i32 as uint32_t |
              0x400000i32 as uint32_t | 0x800000i32 as uint32_t);
    HRTIM_timcr |= (*pTimerCfg).UpdateTrigger;
    /* Enable/Disable the fault channel at timer level */
    HRTIM_timfltr &=
        !(0x1i32 as uint32_t | 0x2i32 as uint32_t | 0x4i32 as uint32_t |
              0x8i32 as uint32_t | 0x10i32 as uint32_t);
    HRTIM_timfltr |=
        (*pTimerCfg).FaultEnable &
            (0x1i32 as uint32_t | 0x2i32 as uint32_t | 0x4i32 as uint32_t |
                 0x8i32 as uint32_t | 0x10i32 as uint32_t);
    /* Lock/Unlock fault sources at timer level */
    HRTIM_timfltr &= !0x80000000u32;
    HRTIM_timfltr |= (*pTimerCfg).FaultLock;
    /* Enable/Disable dead time insertion at timer level */
    HRTIM_timoutr &= !(0x100i32 as uint32_t);
    HRTIM_timoutr |= (*pTimerCfg).DeadTimeInsertion;
    /* Enable/Disable delayed protection at timer level */
    HRTIM_timoutr &= !(0x1c00i32 as uint32_t | 0x200i32 as uint32_t);
    HRTIM_timoutr |= (*pTimerCfg).DelayedProtectionMode;
    /* Set the timer counter reset trigger */
    HRTIM_timrstr = (*pTimerCfg).ResetTrigger;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].TIMxCR
                                    as *mut uint32_t, HRTIM_timcr);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].FLTxR
                                    as *mut uint32_t, HRTIM_timfltr);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].OUTxR
                                    as *mut uint32_t, HRTIM_timoutr);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].RSTxR
                                    as *mut uint32_t, HRTIM_timrstr);
}
/* *
  * @brief  Configures the compare unit of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   0xFF for master timer
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CompareUnit: Compare unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_COMPAREUNIT_1: Compare unit 1
  *                    @arg HRTIM_COMPAREUNIT_2: Compare unit 2
  *                    @arg HRTIM_COMPAREUNIT_3: Compare unit 3
  *                    @arg HRTIM_COMPAREUNIT_4: Compare unit 4
  * @param  pCompareCfg: pointer to the compare unit configuration structure
  * @note When auto delayed mode is required for compare unit 2 or compare unit 4, 
  *       application has to configure separately the capture unit. Capture unit 
  *       to configure in that case depends on the compare unit auto delayed mode
  *       is applied to (see below):
  *         Auto delayed on output compare 2: capture unit 1 must be configured
  *         Auto delayed on output compare 4: capture unit 2 must be configured
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformCompareConfig(mut HRTIMx:
                                                         *mut HRTIM_TypeDef,
                                                     mut TimerIdx: uint32_t,
                                                     mut CompareUnit:
                                                         uint32_t,
                                                     mut pCompareCfg:
                                                         *mut HRTIM_CompareCfgTypeDef) {
    let mut HRTIM_timcr: uint32_t = 0;
    /* Check parameters */
    /* Configure the compare unit */
    match CompareUnit {
        1 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP1xR
                                            as *mut uint32_t,
                                        (*pCompareCfg).CompareValue)
        }
        2 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP2xR
                                            as *mut uint32_t,
                                        (*pCompareCfg).CompareValue);
            if (*pCompareCfg).AutoDelayedMode != 0i32 as uint32_t {
                /* Configure auto-delayed mode */
                HRTIM_timcr =
                    (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxCR;
                HRTIM_timcr &= !(0x3000i32 as uint32_t);
                HRTIM_timcr |= (*pCompareCfg).AutoDelayedMode;
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].TIMxCR
                                                as *mut uint32_t,
                                            HRTIM_timcr);
                /* Set the compare value for timeout compare unit (if any) */
                if (*pCompareCfg).AutoDelayedMode == 0x2000i32 as uint32_t {
                    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                as
                                                                                usize].CMP1xR
                                                    as *mut uint32_t,
                                                (*pCompareCfg).AutoDelayedTimeout)
                } else if (*pCompareCfg).AutoDelayedMode ==
                              0x2000i32 as uint32_t | 0x1000i32 as uint32_t {
                    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                as
                                                                                usize].CMP3xR
                                                    as *mut uint32_t,
                                                (*pCompareCfg).AutoDelayedTimeout)
                }
            }
        }
        4 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP3xR
                                            as *mut uint32_t,
                                        (*pCompareCfg).CompareValue)
        }
        8 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP4xR
                                            as *mut uint32_t,
                                        (*pCompareCfg).CompareValue);
            if (*pCompareCfg).AutoDelayedMode != 0i32 as uint32_t {
                /* Configure auto-delayed mode */
                HRTIM_timcr =
                    (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxCR;
                HRTIM_timcr &= !(0xc000i32 as uint32_t);
                HRTIM_timcr |= (*pCompareCfg).AutoDelayedMode << 2i32;
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].TIMxCR
                                                as *mut uint32_t,
                                            HRTIM_timcr);
                /* Set the compare value for timeout compare unit (if any) */
                if (*pCompareCfg).AutoDelayedMode == 0x2000i32 as uint32_t {
                    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                as
                                                                                usize].CMP1xR
                                                    as *mut uint32_t,
                                                (*pCompareCfg).AutoDelayedTimeout)
                } else if (*pCompareCfg).AutoDelayedMode ==
                              0x2000i32 as uint32_t | 0x1000i32 as uint32_t {
                    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                as
                                                                                usize].CMP3xR
                                                    as *mut uint32_t,
                                                (*pCompareCfg).AutoDelayedTimeout)
                }
            }
        }
        _ => { }
    };
}
/* *
  * @brief  Sets the HRTIMx Master Comparex Register value 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  CompareUnit: Compare unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_COMPAREUNIT_1: Compare unit 1
  *                    @arg HRTIM_COMPAREUNIT_2: Compare unit 2
  *                    @arg HRTIM_COMPAREUNIT_3: Compare unit 3
  *                    @arg HRTIM_COMPAREUNIT_4: Compare unit 4
  * @param  Compare: specifies the Comparex register new value
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_MasterSetCompare(mut HRTIMx:
                                                    *mut HRTIM_TypeDef,
                                                mut CompareUnit: uint32_t,
                                                mut Compare: uint32_t) {
    /* Check parameters */
    /* Configure the compare unit */
    match CompareUnit {
        1 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP1R as
                                            *mut uint32_t, Compare)
        }
        2 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP2R as
                                            *mut uint32_t, Compare)
        }
        4 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP3R as
                                            *mut uint32_t, Compare)
        }
        8 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP4R as
                                            *mut uint32_t, Compare)
        }
        _ => { }
    };
}
/* *
  * @brief  Sets the HRTIMx Slave Comparex Register value 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  CompareUnit: Compare unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_COMPAREUNIT_1: Compare unit 1
  *                    @arg HRTIM_COMPAREUNIT_2: Compare unit 2
  *                    @arg HRTIM_COMPAREUNIT_3: Compare unit 3
  *                    @arg HRTIM_COMPAREUNIT_4: Compare unit 4
  * @param  Compare: specifies the Comparex register new value
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SlaveSetCompare(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut TimerIdx: uint32_t,
                                               mut CompareUnit: uint32_t,
                                               mut Compare: uint32_t) {
    /* Check parameters */
    /* Configure the compare unit */
    match CompareUnit {
        1 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP1xR
                                            as *mut uint32_t, Compare)
        }
        2 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP2xR
                                            as *mut uint32_t, Compare)
        }
        4 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP3xR
                                            as *mut uint32_t, Compare)
        }
        8 => {
            /* Set the compare value */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CMP4xR
                                            as *mut uint32_t, Compare)
        }
        _ => { }
    };
}
/* *
  * @brief  Configures the capture unit of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureChannel: Capture unit to configure
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @param  pCaptureCfg: pointer to the compare unit configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformCaptureConfig(mut HRTIMx:
                                                         *mut HRTIM_TypeDef,
                                                     mut TimerIdx: uint32_t,
                                                     mut CaptureUnit:
                                                         uint32_t,
                                                     mut pCaptureCfg:
                                                         *mut HRTIM_CaptureCfgTypeDef) {
    /* Configure the capture unit */
    match CaptureUnit {
        1 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT1xCR
                                            as *mut uint32_t,
                                        (*pCaptureCfg).Trigger)
        }
        2 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT2xCR
                                            as *mut uint32_t,
                                        (*pCaptureCfg).Trigger)
        }
        _ => { }
    };
}
/* *
  * @brief  Configures the output of a timer operating in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Output: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2 
  * @param  pOutputCfg: pointer to the timer output configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformOutputConfig(mut HRTIMx:
                                                        *mut HRTIM_TypeDef,
                                                    mut TimerIdx: uint32_t,
                                                    mut Output: uint32_t,
                                                    mut pOutputCfg:
                                                        *mut HRTIM_OutputCfgTypeDef) {
    /* Check parameters */
    /* Configure the timer output */
    HRTIM_OutputConfig(HRTIMx, TimerIdx, Output, pOutputCfg);
}
/* *
  * @brief  Configures the event filtering capabilities of a timer (blanking, windowing) 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Event: external event for which timer event filtering must be configured
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_EVENT_1: External event 1
  *                    @arg HRTIM_EVENT_2: External event 2
  *                    @arg HRTIM_EVENT_3: External event 3
  *                    @arg HRTIM_EVENT_4: External event 4
  *                    @arg HRTIM_EVENT_5: External event 5
  *                    @arg HRTIM_EVENT_6: External event 6
  *                    @arg HRTIM_EVENT_7: External event 7
  *                    @arg HRTIM_EVENT_8: External event 8
  *                    @arg HRTIM_EVENT_9: External event 9
  *                    @arg HRTIM_EVENT_10: External event 10
  * @param  pTimerEventFilteringCfg: pointer to the timer event filtering configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_TimerEventFilteringConfig(mut HRTIMx:
                                                             *mut HRTIM_TypeDef,
                                                         mut TimerIdx:
                                                             uint32_t,
                                                         mut Event: uint32_t,
                                                         mut pTimerEventFilteringCfg:
                                                             *mut HRTIM_TimerEventFilteringCfgTypeDef) {
    let mut HRTIM_eefr: uint32_t = 0;
    /* Check parameters */
    /* Configure timer event filtering capabilities */
    match Event {
        0 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR1
                                            as *mut uint32_t,
                                        0i32 as uint32_t);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR2
                                            as *mut uint32_t,
                                        0i32 as uint32_t)
        }
        1 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR1;
            HRTIM_eefr &= !(0x1ei32 as uint32_t | 0x1i32 as uint32_t);
            HRTIM_eefr |=
                (*pTimerEventFilteringCfg).Filter |
                    (*pTimerEventFilteringCfg).Latch;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR1
                                            as *mut uint32_t, HRTIM_eefr)
        }
        2 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR1;
            HRTIM_eefr &= !(0x780i32 as uint32_t | 0x40i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 6i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR1
                                            as *mut uint32_t, HRTIM_eefr)
        }
        4 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR1;
            HRTIM_eefr &= !(0x1e000i32 as uint32_t | 0x1000i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 12i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR1
                                            as *mut uint32_t, HRTIM_eefr)
        }
        8 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR1;
            HRTIM_eefr &= !(0x780000i32 as uint32_t | 0x40000i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 18i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR1
                                            as *mut uint32_t, HRTIM_eefr)
        }
        16 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR1;
            HRTIM_eefr &=
                !(0x1e000000i32 as uint32_t | 0x1000000i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 24i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR1
                                            as *mut uint32_t, HRTIM_eefr)
        }
        32 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR2;
            HRTIM_eefr &= !(0x1ei32 as uint32_t | 0x1i32 as uint32_t);
            HRTIM_eefr |=
                (*pTimerEventFilteringCfg).Filter |
                    (*pTimerEventFilteringCfg).Latch;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR2
                                            as *mut uint32_t, HRTIM_eefr)
        }
        64 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR2;
            HRTIM_eefr &= !(0x780i32 as uint32_t | 0x40i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 6i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR2
                                            as *mut uint32_t, HRTIM_eefr)
        }
        128 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR2;
            HRTIM_eefr &= !(0x1e000i32 as uint32_t | 0x1000i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 12i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR2
                                            as *mut uint32_t, HRTIM_eefr)
        }
        256 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR2;
            HRTIM_eefr &= !(0x780000i32 as uint32_t | 0x40000i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 18i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR2
                                            as *mut uint32_t, HRTIM_eefr)
        }
        512 => {
            HRTIM_eefr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].EEFxR2;
            HRTIM_eefr &=
                !(0x1e000000i32 as uint32_t | 0x1000000i32 as uint32_t);
            HRTIM_eefr |=
                ((*pTimerEventFilteringCfg).Filter |
                     (*pTimerEventFilteringCfg).Latch) << 24i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].EEFxR2
                                            as *mut uint32_t, HRTIM_eefr)
        }
        _ => { }
    };
}
/* *
  * @brief  Configures the dead time insertion feature for a timer 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  pDeadTimeCfg: pointer to the dead time insertion configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_DeadTimeConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut TimerIdx: uint32_t,
                                              mut pDeadTimeCfg:
                                                  *mut HRTIM_DeadTimeCfgTypeDef) {
    let mut HRTIM_dtr: uint32_t = 0;
    /* Check parameters */
    HRTIM_dtr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].DTxR;
    /* Clear timer dead times configuration */
    HRTIM_dtr &=
        !(0x1ffi32 as uint32_t | 0x200i32 as uint32_t | 0x1c00i32 as uint32_t
              | 0x4000i32 as uint32_t | 0x8000i32 as uint32_t |
              0x2000000i32 as uint32_t | 0x200i32 as uint32_t |
              0x40000000i32 as uint32_t | 0x80000000u32);
    /* Set timer dead times configuration */
    HRTIM_dtr |= (*pDeadTimeCfg).Prescaler << 10i32;
    HRTIM_dtr |= (*pDeadTimeCfg).RisingValue;
    HRTIM_dtr |= (*pDeadTimeCfg).RisingSign;
    HRTIM_dtr |= (*pDeadTimeCfg).RisingSignLock;
    HRTIM_dtr |= (*pDeadTimeCfg).RisingLock;
    HRTIM_dtr |= (*pDeadTimeCfg).FallingValue << 16i32;
    HRTIM_dtr |= (*pDeadTimeCfg).FallingSign;
    HRTIM_dtr |= (*pDeadTimeCfg).FallingSignLock;
    HRTIM_dtr |= (*pDeadTimeCfg).FallingLock;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].DTxR as
                                    *mut uint32_t, HRTIM_dtr);
}
/* *
  * @brief  Configures the chopper mode feature for a timer 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  pChopperModeCfg: pointer to the chopper mode configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ChopperModeConfig(mut HRTIMx:
                                                     *mut HRTIM_TypeDef,
                                                 mut TimerIdx: uint32_t,
                                                 mut pChopperModeCfg:
                                                     *mut HRTIM_ChopperModeCfgTypeDef) {
    let mut HRTIM_chpr: uint32_t = 0;
    /* Check parameters */
    HRTIM_chpr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].CHPxR;
    /* Clear timer chopper mode configuration */
    HRTIM_chpr &=
        !(0xfi32 as uint32_t | 0x70i32 as uint32_t | 0x780i32 as uint32_t);
    /* Set timer chopper mode configuration */
    HRTIM_chpr |= (*pChopperModeCfg).CarrierFreq;
    HRTIM_chpr |= (*pChopperModeCfg).DutyCycle << 4i32;
    HRTIM_chpr |= (*pChopperModeCfg).StartPulse << 7i32;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].CHPxR
                                    as *mut uint32_t, HRTIM_chpr);
}
/* *
  * @brief  Configures the burst DMA controller for a timer 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
   *                  This parameter can be one of the following values:
 *                    @arg 0x5 for master timer
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  RegistersToUpdate: registers to be written by DMA
  *                    This parameter can be any combination of the following values:
  *                    @arg HRTIM_BURSTDMA_CR: HRTIM_MCR or HRTIM_TIMxCR
  *                    @arg HRTIM_BURSTDMA_ICR: HRTIM_MICR or HRTIM_TIMxICR
  *                    @arg HRTIM_BURSTDMA_DIER: HRTIM_MDIER or HRTIM_TIMxDIER
  *                    @arg HRTIM_BURSTDMA_CNT: HRTIM_MCNT or HRTIM_TIMxCNT
  *                    @arg HRTIM_BURSTDMA_PER: HRTIM_MPER or HRTIM_TIMxPER
  *                    @arg HRTIM_BURSTDMA_REP: HRTIM_MREP or HRTIM_TIMxREP
  *                    @arg HRTIM_BURSTDMA_CMP1: HRTIM_MCMP1 or HRTIM_TIMxCMP1
  *                    @arg HRTIM_BURSTDMA_CMP2: HRTIM_MCMP2 or HRTIM_TIMxCMP2
  *                    @arg HRTIM_BURSTDMA_CMP3: HRTIM_MCMP3 or HRTIM_TIMxCMP3
  *                    @arg HRTIM_BURSTDMA_CMP4: HRTIM_MCMP4 or HRTIM_TIMxCMP4
  *                    @arg HRTIM_BURSTDMA_DTR: HRTIM_TIMxDTR
  *                    @arg HRTIM_BURSTDMA_SET1R: HRTIM_TIMxSET1R
  *                    @arg HRTIM_BURSTDMA_RST1R: HRTIM_TIMxRST1R
  *                    @arg HRTIM_BURSTDMA_SET2R: HRTIM_TIMxSET2R
  *                    @arg HRTIM_BURSTDMA_RST2R: HRTIM_TIMxRST2R
  *                    @arg HRTIM_BURSTDMA_EEFR1: HRTIM_TIMxEEFR1
  *                    @arg HRTIM_BURSTDMA_EEFR2: HRTIM_TIMxEEFR2
  *                    @arg HRTIM_BURSTDMA_RSTR: HRTIM_TIMxRSTR
  *                    @arg HRTIM_BURSTDMA_CHPR: HRTIM_TIMxCHPR
  *                    @arg HRTIM_BURSTDMA_OUTR: HRTIM_TIMxOUTR
  *                    @arg HRTIM_BURSTDMA_FLTR: HRTIM_TIMxFLTR
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_BurstDMAConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut TimerIdx: uint32_t,
                                              mut RegistersToUpdate:
                                                  uint32_t) {
    /* Check parameters */
    /* Set the burst DMA timer update register */
    match TimerIdx {
        0 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BDTAUPR as
                                            *mut uint32_t, RegistersToUpdate)
        }
        1 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BDTBUPR as
                                            *mut uint32_t, RegistersToUpdate)
        }
        2 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BDTCUPR as
                                            *mut uint32_t, RegistersToUpdate)
        }
        3 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BDTDUPR as
                                            *mut uint32_t, RegistersToUpdate)
        }
        4 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BDTEUPR as
                                            *mut uint32_t, RegistersToUpdate)
        }
        5 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BDMUPDR as
                                            *mut uint32_t, RegistersToUpdate)
        }
        _ => { }
    };
}
/* *
  * @brief  Configures the external input/output synchronization of the HRTIMx 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  pSynchroCfg: pointer to the input/output synchronization configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SynchronizationConfig(mut HRTIMx:
                                                         *mut HRTIM_TypeDef,
                                                     mut pSynchroCfg:
                                                         *mut HRTIM_SynchroCfgTypeDef) {
    let mut HRTIM_mcr: uint32_t = 0;
    /* Check parameters */
    HRTIM_mcr = (*HRTIMx).HRTIM_MASTER.MCR;
    /* Set the synchronization input source */
    HRTIM_mcr &= !(0x300i32 as uint32_t);
    HRTIM_mcr |= (*pSynchroCfg).SyncInputSource;
    /* Set the event to be sent on the synchronization output */
    HRTIM_mcr &= !(0xc000i32 as uint32_t);
    HRTIM_mcr |= (*pSynchroCfg).SyncOutputSource;
    /* Set the polarity of the synchronization output */
    HRTIM_mcr &= !(0x3000i32 as uint32_t);
    HRTIM_mcr |= (*pSynchroCfg).SyncOutputPolarity;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t, HRTIM_mcr);
}
/* *
  * @brief  Configures the burst mode feature of the HRTIMx 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  pBurstModeCfg: pointer to the burst mode configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_BurstModeConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut pBurstModeCfg:
                                                   *mut HRTIM_BurstModeCfgTypeDef) {
    let mut HRTIM_bmcr: uint32_t = 0;
    /* Check parameters */
    HRTIM_bmcr = (*HRTIMx).HRTIM_COMMON.BMCR;
    /* Set the burst mode operating mode */
    HRTIM_bmcr &= !(0x2i32 as uint32_t);
    HRTIM_bmcr |= (*pBurstModeCfg).Mode;
    /* Set the burst mode clock source */
    HRTIM_bmcr &= !(0x3ci32 as uint32_t);
    HRTIM_bmcr |= (*pBurstModeCfg).ClockSource;
    /* Set the burst mode prescaler */
    HRTIM_bmcr &= !(0x3c0i32 as uint32_t);
    HRTIM_bmcr |= (*pBurstModeCfg).Prescaler;
    /* Enable/disable burst mode registers preload */
    HRTIM_bmcr &= !(0x400i32 as uint32_t);
    HRTIM_bmcr |= (*pBurstModeCfg).PreloadEnable;
    /* Set the burst mode trigger */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMTRGR as
                                    *mut uint32_t, (*pBurstModeCfg).Trigger);
    /* Set the burst mode compare value */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMCMPR as
                                    *mut uint32_t,
                                (*pBurstModeCfg).IdleDuration);
    /* Set the burst mode period */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMPER as
                                    *mut uint32_t, (*pBurstModeCfg).Period);
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMCR as
                                    *mut uint32_t, HRTIM_bmcr);
}
/* *
  * @brief  Configures the conditioning of an external event
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Event: external event to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_EVENT_1: External event 1
  *                    @arg HRTIM_EVENT_2: External event 2
  *                    @arg HRTIM_EVENT_3: External event 3
  *                    @arg HRTIM_EVENT_4: External event 4
  *                    @arg HRTIM_EVENT_5: External event 5
  *                    @arg HRTIM_EVENT_6: External event 6
  *                    @arg HRTIM_EVENT_7: External event 7
  *                    @arg HRTIM_EVENT_8: External event 8
  *                    @arg HRTIM_EVENT_9: External event 9
  *                    @arg HRTIM_EVENT_10: External event 10
  * @param  pEventCfg: pointer to the event conditioning configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_EventConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                           mut Event: uint32_t,
                                           mut pEventCfg:
                                               *mut HRTIM_EventCfgTypeDef) {
    /* Check parameters */
    /* Configure the event channel */
    HRTIM_ExternalEventConfig(HRTIMx, Event, pEventCfg);
}
/* *
  * @brief  Configures the external event conditioning block prescaler
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Prescaler: Prescaler value
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_EVENTPRESCALER_DIV1: fEEVS=fHRTIMx
  *                    @arg HRTIM_EVENTPRESCALER_DIV2: fEEVS=fHRTIMx / 2
  *                    @arg HRTIM_EVENTPRESCALER_DIV4: fEEVS=fHRTIMx / 4
  *                    @arg HRTIM_EVENTPRESCALER_DIV8: fEEVS=fHRTIMx / 8
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_EventPrescalerConfig(mut HRTIMx:
                                                        *mut HRTIM_TypeDef,
                                                    mut Prescaler: uint32_t) {
    let mut HRTIM_eecr3: uint32_t = 0;
    /* Check parameters */
    /* Set the external event prescaler */
    HRTIM_eecr3 = (*HRTIMx).HRTIM_COMMON.EECR3;
    HRTIM_eecr3 &= !0xc0000000u32;
    HRTIM_eecr3 |= Prescaler;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR3 as
                                    *mut uint32_t, HRTIM_eecr3);
}
/* *
  * @brief  Configures the conditioning of fault input
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Fault: fault input to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_FAULT_1: Fault input 1
  *                    @arg HRTIM_FAULT_2: Fault input 2
  *                    @arg HRTIM_FAULT_3: Fault input 3
  *                    @arg HRTIM_FAULT_4: Fault input 4
  *                    @arg HRTIM_FAULT_5: Fault input 5
  * @param  pFaultCfg: pointer to the fault conditioning configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_FaultConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                           mut pFaultCfg:
                                               *mut HRTIM_FaultCfgTypeDef,
                                           mut Fault: uint32_t) {
    let mut HRTIM_fltinr1: uint32_t = 0;
    let mut HRTIM_fltinr2: uint32_t = 0;
    /* Check parameters */
    /* Configure fault channel */
    HRTIM_fltinr1 = (*HRTIMx).HRTIM_COMMON.FLTINxR1;
    HRTIM_fltinr2 = (*HRTIMx).HRTIM_COMMON.FLTINxR2;
    match Fault {
        1 => {
            HRTIM_fltinr1 &=
                !(0x2i32 as uint32_t | 0x4i32 as uint32_t |
                      0x78i32 as uint32_t | 0x80i32 as uint32_t);
            HRTIM_fltinr1 |= (*pFaultCfg).Polarity;
            HRTIM_fltinr1 |= (*pFaultCfg).Source;
            HRTIM_fltinr1 |= (*pFaultCfg).Filter;
            HRTIM_fltinr1 |= (*pFaultCfg).Lock
        }
        2 => {
            HRTIM_fltinr1 &=
                !(0x200i32 as uint32_t | 0x400i32 as uint32_t |
                      0x7800i32 as uint32_t | 0x8000i32 as uint32_t);
            HRTIM_fltinr1 |= (*pFaultCfg).Polarity << 8i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Source << 8i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Filter << 8i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Lock << 8i32
        }
        4 => {
            HRTIM_fltinr1 &=
                !(0x20000i32 as uint32_t | 0x40000i32 as uint32_t |
                      0x780000i32 as uint32_t | 0x800000i32 as uint32_t);
            HRTIM_fltinr1 |= (*pFaultCfg).Polarity << 16i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Source << 16i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Filter << 16i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Lock << 16i32
        }
        8 => {
            HRTIM_fltinr1 &=
                !(0x2000000i32 as uint32_t | 0x4000000i32 as uint32_t |
                      0x78000000i32 as uint32_t | 0x80000000u32);
            HRTIM_fltinr1 |= (*pFaultCfg).Polarity << 24i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Source << 24i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Filter << 24i32;
            HRTIM_fltinr1 |= (*pFaultCfg).Lock << 24i32
        }
        16 => {
            HRTIM_fltinr2 &=
                !(0x2i32 as uint32_t | 0x4i32 as uint32_t |
                      0x78i32 as uint32_t | 0x80i32 as uint32_t);
            HRTIM_fltinr2 |= (*pFaultCfg).Polarity;
            HRTIM_fltinr2 |= (*pFaultCfg).Source;
            HRTIM_fltinr2 |= (*pFaultCfg).Filter;
            HRTIM_fltinr2 |= (*pFaultCfg).Lock
        }
        _ => { }
    }
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.FLTINxR1 as
                                    *mut uint32_t, HRTIM_fltinr1);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.FLTINxR2 as
                                    *mut uint32_t, HRTIM_fltinr2);
}
/* *
  * @brief  Configures the fault conditioning block prescaler
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Prescaler: Prescaler value
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_FAULTPRESCALER_DIV1: fFLTS=fHRTIMx
  *                    @arg HRTIM_FAULTPRESCALER_DIV2: fFLTS=fHRTIMx / 2
  *                    @arg HRTIM_FAULTPRESCALER_DIV4: fFLTS=fHRTIMx / 4
  *                    @arg HRTIM_FAULTPRESCALER_DIV8: fFLTS=fHRTIMx / 8
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_FaultPrescalerConfig(mut HRTIMx:
                                                        *mut HRTIM_TypeDef,
                                                    mut Prescaler: uint32_t) {
    let mut HRTIM_fltinr2: uint32_t = 0;
    /* Check parameters */
    /* Set the external event prescaler */
    HRTIM_fltinr2 = (*HRTIMx).HRTIM_COMMON.FLTINxR2;
    HRTIM_fltinr2 &= !(0x3000000i32 as uint32_t);
    HRTIM_fltinr2 |= Prescaler;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.FLTINxR2 as
                                    *mut uint32_t, HRTIM_fltinr2);
}
/* *
  * @brief  Enables or disables the HRTIMx Fault mode.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Fault: fault input to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_FAULT_1: Fault input 1
  *                    @arg HRTIM_FAULT_2: Fault input 2
  *                    @arg HRTIM_FAULT_3: Fault input 3
  *                    @arg HRTIM_FAULT_4: Fault input 4
  *                    @arg HRTIM_FAULT_5: Fault input 5
  * @param  Enable: Fault mode controller enabling
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_FAULT_ENABLED: Fault mode enabled
  *                    @arg HRTIM_FAULT_DISABLED: Fault mode disabled
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_FaultModeCtl(mut HRTIMx: *mut HRTIM_TypeDef,
                                            mut Fault: uint32_t,
                                            mut Enable: uint32_t) {
    let mut HRTIM_fltinr1: uint32_t = 0;
    let mut HRTIM_fltinr2: uint32_t = 0;
    /* Check parameters */
    /* Configure fault channel */
    HRTIM_fltinr1 = (*HRTIMx).HRTIM_COMMON.FLTINxR1;
    HRTIM_fltinr2 = (*HRTIMx).HRTIM_COMMON.FLTINxR2;
    match Fault {
        1 => {
            HRTIM_fltinr1 &= !(0x1i32 as uint32_t);
            HRTIM_fltinr1 |= Enable
        }
        2 => {
            HRTIM_fltinr1 &= !(0x100i32 as uint32_t);
            HRTIM_fltinr1 |= Enable << 8i32
        }
        4 => {
            HRTIM_fltinr1 &= !(0x10000i32 as uint32_t);
            HRTIM_fltinr1 |= Enable << 16i32
        }
        8 => {
            HRTIM_fltinr1 &= !(0x1000000i32 as uint32_t);
            HRTIM_fltinr1 |= Enable << 24i32
        }
        16 => {
            HRTIM_fltinr2 &= !(0x1i32 as uint32_t);
            HRTIM_fltinr2 |= Enable
        }
        _ => { }
    }
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.FLTINxR1 as
                                    *mut uint32_t, HRTIM_fltinr1);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.FLTINxR2 as
                                    *mut uint32_t, HRTIM_fltinr2);
}
/* *
  * @brief  Configures both the ADC trigger register update source and the ADC
  *         trigger source.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  ADC trigger: ADC trigger to configure
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_ADCTRIGGER_1: ADC trigger 1
  *                    @arg HRTIM_ADCTRIGGER_2: ADC trigger 2
  *                    @arg HRTIM_ADCTRIGGER_3: ADC trigger 3
  *                    @arg HRTIM_ADCTRIGGER_4: ADC trigger 4
  * @param  pADCTriggerCfg: pointer to the ADC trigger configuration structure
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_ADCTriggerConfig(mut HRTIMx:
                                                    *mut HRTIM_TypeDef,
                                                mut ADCTrigger: uint32_t,
                                                mut pADCTriggerCfg:
                                                    *mut HRTIM_ADCTriggerCfgTypeDef) {
    let mut HRTIM_cr1: uint32_t = 0;
    /* Check parameters */
    /* Set the ADC trigger update source */
    HRTIM_cr1 = (*HRTIMx).HRTIM_COMMON.CR1;
    let mut current_block_18: u64;
    match ADCTrigger {
        1 => {
            HRTIM_cr1 &= !(0x70000i32 as uint32_t);
            HRTIM_cr1 |= (*pADCTriggerCfg).UpdateSource;
            /* Set the ADC trigger 1 source */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.ADC1R as
                                            *mut uint32_t,
                                        (*pADCTriggerCfg).Trigger);
            current_block_18 = 224731115979188411;
        }
        2 => {
            HRTIM_cr1 &= !(0x380000i32 as uint32_t);
            HRTIM_cr1 |= (*pADCTriggerCfg).UpdateSource << 3i32;
            /* Set the ADC trigger 2 source */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.ADC2R as
                                            *mut uint32_t,
                                        (*pADCTriggerCfg).Trigger);
            current_block_18 = 224731115979188411;
        }
        4 => {
            HRTIM_cr1 &= !(0x1c00000i32 as uint32_t);
            HRTIM_cr1 |= (*pADCTriggerCfg).UpdateSource << 6i32;
            /* Set the ADC trigger 3 source */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.ADC3R as
                                            *mut uint32_t,
                                        (*pADCTriggerCfg).Trigger);
            current_block_18 = 13144893622685093897;
        }
        8 => { current_block_18 = 13144893622685093897; }
        _ => { current_block_18 = 224731115979188411; }
    }
    match current_block_18 {
        13144893622685093897 => {
            HRTIM_cr1 &= !(0xe000000i32 as uint32_t);
            HRTIM_cr1 |= (*pADCTriggerCfg).UpdateSource << 9i32;
            /* Set the ADC trigger 4 source */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.ADC4R as
                                            *mut uint32_t,
                                        (*pADCTriggerCfg).Trigger)
        }
        _ => { }
    }
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.CR1 as
                                    *mut uint32_t, HRTIM_cr1);
}
/* *
  * @brief  Enables or disables the HRTIMx burst mode controller.
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Enable: Burst mode controller enabling
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_BURSTMODECTL_ENABLED: Burst mode enabled
  *                    @arg HRTIM_BURSTMODECTL_DISABLED: Burst mode disabled
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_BurstModeCtl(mut HRTIMx: *mut HRTIM_TypeDef,
                                            mut Enable: uint32_t) {
    let mut HRTIM_bmcr: uint32_t = 0;
    /* Check parameters */
    /* Enable/Disable the burst mode controller */
    HRTIM_bmcr = (*HRTIMx).HRTIM_COMMON.BMCR;
    HRTIM_bmcr &= !(0x1i32 as uint32_t);
    HRTIM_bmcr |= Enable;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMCR as
                                    *mut uint32_t, HRTIM_bmcr);
}
/* *
  * @brief  Triggers a software capture on the designed capture unit
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureUnit: Capture unit to trig
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @retval None
  * @note The 'software capture' bit in the capure configuration register is
  *       automatically reset by hardware
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SoftwareCapture(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut TimerIdx: uint32_t,
                                               mut CaptureUnit: uint32_t) {
    /* Check parameters */
    /* Force a software capture on concerned capture unit */
    match CaptureUnit {
        1 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT1xCR
                                            as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                            as
                                                                                                            usize].CPT1xCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1i32 as uint32_t) as uint32_t
                                            as uint32_t)
        }
        2 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT2xCR
                                            as *mut uint32_t,
                                        (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                            as
                                                                                                            usize].CPT2xCR
                                                                                    as
                                                                                    *const uint32_t)
                                             as libc::c_uint |
                                             0x1i32 as uint32_t) as uint32_t
                                            as uint32_t)
        }
        _ => { }
    };
}
/* *
  * @brief  Triggers the update of the registers of one or several timers
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimersToUpdate: timers concerned with the software register update
  *                   This parameter can be any combination of the following values:
  *                   @arg HRTIM_TIMERUPDATE_MASTER 
  *                   @arg HRTIM_TIMERUPDATE_A 
  *                   @arg HRTIM_TIMERUPDATE_B 
  *                   @arg HRTIM_TIMERUPDATE_C 
  *                   @arg HRTIM_TIMERUPDATE_D 
  *                   @arg HRTIM_TIMERUPDATE_E 
  * @retval None
  * @note The 'software update' bits in the HRTIMx control register 2 register are
  *       automatically reset by hardware
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SoftwareUpdate(mut HRTIMx: *mut HRTIM_TypeDef,
                                              mut TimersToUpdate: uint32_t) {
    /* Check parameters */
    /* Force timer(s) registers update */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | TimersToUpdate) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Triggers the reset of one or several timers
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimersToUpdate: timers concerned with the software counter reset
  *                   This parameter can be any combination of the following values:
  *                   @arg HRTIM_TIMER_MASTER 
  *                   @arg HRTIM_TIMER_A 
  *                   @arg HRTIM_TIMER_B 
  *                   @arg HRTIM_TIMER_C 
  *                   @arg HRTIM_TIMER_D 
  *                   @arg HRTIM_TIMER_E 
  * @retval None
  * @note The 'software reset' bits in the HRTIMx control register 2  are
  *       automatically reset by hardware
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_SoftwareReset(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimersToReset: uint32_t) {
    /* Check parameters */
    /* Force timer(s) registers update */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.CR2 as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_COMMON.CR2
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint | TimersToReset) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Forces the timer output to its active or inactive state 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Output: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @param OutputLevel: indicates whether the output is forced to its active or inactive state
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUTLEVEL_ACTIVE: output is forced to its active state
  *                    @arg HRTIM_OUTPUTLEVEL_INACTIVE: output is forced to its inactive state
  * @retval None
  * @note The 'software set/reset trigger' bit in the output set/reset registers 
  *       is automatically reset by hardware
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformSetOutputLevel(mut HRTIMx:
                                                          *mut HRTIM_TypeDef,
                                                      mut TimerIdx: uint32_t,
                                                      mut Output: uint32_t,
                                                      mut OutputLevel:
                                                          uint32_t) {
    /* Check parameters */
    /* Force timer output level */
    match Output {
        1 | 4 | 16 | 64 | 256 => {
            if OutputLevel == 0x1i32 as uint32_t {
                /* Force output to its active state */
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].SETx1R
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].SETx1R
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1i32 as uint32_t) as
                                                uint32_t as uint32_t)
            } else {
                /* Force output to its inactive state */
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].RSTx1R
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].RSTx1R
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1i32 as uint32_t) as
                                                uint32_t as uint32_t)
            }
        }
        2 | 8 | 32 | 128 | 512 => {
            if OutputLevel == 0x1i32 as uint32_t {
                /* Force output to its active state */
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].SETx2R
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].SETx2R
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1i32 as uint32_t) as
                                                uint32_t as uint32_t)
            } else {
                /* Force output to its inactive state */
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].RSTx2R
                                                as *mut uint32_t,
                                            (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                                as
                                                                                                                usize].RSTx2R
                                                                                        as
                                                                                        *const uint32_t)
                                                 as libc::c_uint |
                                                 0x1i32 as uint32_t) as
                                                uint32_t as uint32_t)
            }
        }
        _ => { }
    };
}
/* *
  * @}
  */
/* * @defgroup HRTIM_Group4 Peripheral State methods 
 *  @brief   Peripheral State functions 
 *
@verbatim   
 ===============================================================================
                      ##### Peripheral State methods #####
 ===============================================================================  
    [..]
    This subsection permit to get in run-time the status of the peripheral 
    and the data flow.

@endverbatim
  * @{
  */
/* *
  * @brief  Returns actual value of the capture register of the designated capture unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  CaptureUnit: Capture unit to trig
  *                    This parameter can be one of the following values: 
  *                    @arg HRTIM_CAPTUREUNIT_1: Capture unit 1
  *                    @arg HRTIM_CAPTUREUNIT_2: Capture unit 2
  * @retval Captured value
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetCapturedValue(mut HRTIMx:
                                                    *mut HRTIM_TypeDef,
                                                mut TimerIdx: uint32_t,
                                                mut CaptureUnit: uint32_t)
 -> uint32_t {
    let mut captured_value: uint32_t = 0i32 as uint32_t;
    /* Check parameters */
    /* Read captured value */
    match CaptureUnit {
        1 => {
            captured_value = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].CPT1xR
        }
        2 => {
            captured_value = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].CPT2xR
        }
        _ => { }
    }
    return captured_value;
}
/* *
  * @brief  Returns actual level (active or inactive) of the designated output 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Output: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval Output level
  * @note Returned output level is taken before the output stage (chopper, 
  *        polarity).
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformGetOutputLevel(mut HRTIMx:
                                                          *mut HRTIM_TypeDef,
                                                      mut TimerIdx: uint32_t,
                                                      mut Output: uint32_t)
 -> uint32_t {
    let mut output_level: uint32_t = 0x2i32 as uint32_t;
    /* Check parameters */
    /* Read the output level */
    match Output {
        1 | 4 | 16 | 64 | 256 => {
            if (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR &
                   0x100000i32 as uint32_t !=
                   RESET as libc::c_int as libc::c_uint {
                output_level = 0x1i32 as uint32_t
            } else { output_level = 0x2i32 as uint32_t }
        }
        2 | 8 | 32 | 128 | 512 => {
            if (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR &
                   0x200000i32 as uint32_t !=
                   RESET as libc::c_int as libc::c_uint {
                output_level = 0x1i32 as uint32_t
            } else { output_level = 0x2i32 as uint32_t }
        }
        _ => { }
    }
    return output_level;
}
/* *
  * @brief  Returns actual state (RUN, IDLE, FAULT) of the designated output 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Output: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TE1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TE2: Timer E - Output 2
  * @retval Output state
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_WaveformGetOutputState(mut HRTIMx:
                                                          *mut HRTIM_TypeDef,
                                                      mut TimerIdx: uint32_t,
                                                      mut Output: uint32_t)
 -> uint32_t {
    let mut output_bit: uint32_t = 0i32 as uint32_t;
    let mut output_state: uint32_t = 0x1i32 as uint32_t;
    /* Check parameters */
    /* Set output state according to output control status and output disable status */
    match Output {
        1 => { output_bit = 0x1i32 as uint32_t }
        2 => { output_bit = 0x2i32 as uint32_t }
        4 => { output_bit = 0x4i32 as uint32_t }
        8 => { output_bit = 0x8i32 as uint32_t }
        16 => { output_bit = 0x10i32 as uint32_t }
        32 => { output_bit = 0x20i32 as uint32_t }
        64 => { output_bit = 0x40i32 as uint32_t }
        128 => { output_bit = 0x80i32 as uint32_t }
        256 => { output_bit = 0x100i32 as uint32_t }
        512 => { output_bit = 0x200i32 as uint32_t }
        _ => { }
    }
    if (*HRTIMx).HRTIM_COMMON.OENR & output_bit !=
           RESET as libc::c_int as libc::c_uint {
        /* Output is enabled: output in RUN state (whatever ouput disable status is)*/
        output_state = 0x2i32 as uint32_t
    } else if (*HRTIMx).HRTIM_COMMON.ODSR & output_bit !=
                  RESET as libc::c_int as libc::c_uint {
        /* Output is disabled: output in FAULT state */
        output_state = 0x3i32 as uint32_t
    } else {
        /* Output is disabled: output in IDLE state */
        output_state = 0x1i32 as uint32_t
    }
    return output_state;
}
/* *
  * @brief  Returns the level (active or inactive) of the designated output 
  *         when the delayed protection was triggered 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @param  Output: Timer output
  *                    This parameter can be one of the following values:
  *                    @arg HRTIM_OUTPUT_TA1: Timer A - Output 1
  *                    @arg HRTIM_OUTPUT_TA2: Timer A - Output 2
  *                    @arg HRTIM_OUTPUT_TB1: Timer B - Output 1
  *                    @arg HRTIM_OUTPUT_TB2: Timer B - Output 2
  *                    @arg HRTIM_OUTPUT_TC1: Timer C - Output 1
  *                    @arg HRTIM_OUTPUT_TC2: Timer C - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer D - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer D - Output 2
  *                    @arg HRTIM_OUTPUT_TD1: Timer E - Output 1
  *                    @arg HRTIM_OUTPUT_TD2: Timer E - Output 2
  * @retval Delayed protection status 
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetDelayedProtectionStatus(mut HRTIMx:
                                                              *mut HRTIM_TypeDef,
                                                          mut TimerIdx:
                                                              uint32_t,
                                                          mut Output:
                                                              uint32_t)
 -> uint32_t {
    let mut delayed_protection_status: uint32_t = 0x2i32 as uint32_t;
    /* Check parameters */
    /* Read the delayed protection status */
    match Output {
        1 | 4 | 16 | 64 | 256 => {
            if (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR &
                   0x40000i32 as uint32_t !=
                   RESET as libc::c_int as libc::c_uint {
                /* Output 1 was active when the delayed idle protection was triggered */
                delayed_protection_status = 0x1i32 as uint32_t
            } else {
                /* Output 1 was inactive when the delayed idle protection was triggered */
                delayed_protection_status = 0x2i32 as uint32_t
            }
        }
        2 | 8 | 32 | 128 | 512 => {
            if (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR &
                   0x80000i32 as uint32_t !=
                   RESET as libc::c_int as libc::c_uint {
                /* Output 2 was active when the delayed idle protection was triggered */
                delayed_protection_status = 0x1i32 as uint32_t
            } else {
                /* Output 2 was inactive when the delayed idle protection was triggered */
                delayed_protection_status = 0x2i32 as uint32_t
            }
        }
        _ => { }
    }
    return delayed_protection_status;
}
/* *
  * @brief  Returns the actual status (active or inactive) of the burst mode controller 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @retval Burst mode controller status 
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetBurstStatus(mut HRTIMx: *mut HRTIM_TypeDef)
 -> uint32_t {
    let mut burst_mode_status: uint32_t = 0;
    /* Read burst mode status */
    burst_mode_status = (*HRTIMx).HRTIM_COMMON.BMCR & 0x80000000u32;
    return burst_mode_status;
}
/* *
  * @brief  Indicates on which output the signal is currently active (when the
  *         push pull mode is enabled)
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @retval Burst mode controller status 
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetCurrentPushPullStatus(mut HRTIMx:
                                                            *mut HRTIM_TypeDef,
                                                        mut TimerIdx:
                                                            uint32_t)
 -> uint32_t {
    let mut current_pushpull_status: uint32_t = 0;
    /* Check the parameters */
    /* Read current push pull status */
    current_pushpull_status =
        (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR &
            0x10000i32 as uint32_t;
    return current_pushpull_status;
}
/* Exported functions --------------------------------------------------------*/
/* Simple time base related functions  *****************************************/
/* Simple output compare related functions  ************************************/
/* Simple PWM output related functions  ****************************************/
/* Simple capture related functions  *******************************************/
/* SImple one pulse related functions  *****************************************/
/* Waveform related functions *************************************************/
/* Interrupt/flags and DMA management */
/* *
  * @brief  Indicates on which output the signal was applied, in push-pull mode
            balanced fault mode or delayed idle mode, when the protection was triggered
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  *                   This parameter can be one of the following values:
  *                   @arg 0x0 to 0x4 for timers A to E 
  * @retval Idle Push Pull Status 
  */
#[no_mangle]
pub unsafe extern "C" fn HRTIM_GetIdlePushPullStatus(mut HRTIMx:
                                                         *mut HRTIM_TypeDef,
                                                     mut TimerIdx: uint32_t)
 -> uint32_t {
    let mut idle_pushpull_status: uint32_t = 0;
    /* Check the parameters */
    /* Read current push pull status */
    idle_pushpull_status =
        (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxISR &
            0x20000i32 as uint32_t;
    return idle_pushpull_status;
}
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* *
  * @brief  Configures the master timer time base
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @retval None
  */
unsafe extern "C" fn HRTIM_MasterBase_Config(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut HRTIM_BaseInitStruct:
                                                 *mut HRTIM_BaseInitTypeDef) {
    /* Set the prescaler ratio */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x7i32 as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*HRTIM_BaseInitStruct).PrescalerRatio)
                                    as uint32_t as uint32_t);
    /* Set the operating mode */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x8i32 as uint32_t |
                                           0x10i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_MASTER.MCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*HRTIM_BaseInitStruct).Mode) as uint32_t
                                    as uint32_t);
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MPER as
                                    *mut uint32_t,
                                (*HRTIM_BaseInitStruct).Period);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MREP as
                                    *mut uint32_t,
                                (*HRTIM_BaseInitStruct).RepetitionCounter);
}
/* *
  * @brief  Configures timing unit (timer A to timer E) time base
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @retval None
  */
unsafe extern "C" fn HRTIM_TimingUnitBase_Config(mut HRTIMx:
                                                     *mut HRTIM_TypeDef,
                                                 mut TimerIdx: uint32_t,
                                                 mut HRTIM_BaseInitStruct:
                                                     *mut HRTIM_BaseInitTypeDef) {
    /* Set the prescaler ratio */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].TIMxCR
                                    as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                    as
                                                                                                    usize].TIMxCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint & !(0x7i32 as uint32_t))
                                    as uint32_t as uint32_t);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].TIMxCR
                                    as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                    as
                                                                                                    usize].TIMxCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*HRTIM_BaseInitStruct).PrescalerRatio)
                                    as uint32_t as uint32_t);
    /* Set the operating mode */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].TIMxCR
                                    as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                    as
                                                                                                    usize].TIMxCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint &
                                     !(0x8i32 as uint32_t |
                                           0x10i32 as uint32_t)) as uint32_t
                                    as uint32_t);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].TIMxCR
                                    as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&(*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                                                    as
                                                                                                    usize].TIMxCR
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     (*HRTIM_BaseInitStruct).Mode) as uint32_t
                                    as uint32_t);
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].PERxR
                                    as *mut uint32_t,
                                (*HRTIM_BaseInitStruct).Period);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].REPxR
                                    as *mut uint32_t,
                                (*HRTIM_BaseInitStruct).RepetitionCounter);
}
/* *
  * @brief  Configures the master timer in waveform mode
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  pTimerInit: pointer to the timer initialization data structure
  * @retval None
  */
unsafe extern "C" fn HRTIM_MasterWaveform_Config(mut HRTIMx:
                                                     *mut HRTIM_TypeDef,
                                                 mut pTimerInit:
                                                     *mut HRTIM_TimerInitTypeDef) {
    let mut HRTIM_mcr: uint32_t = 0;
    let mut HRTIM_bmcr: uint32_t = 0;
    /* Configure master timer */
    HRTIM_mcr = (*HRTIMx).HRTIM_MASTER.MCR;
    HRTIM_bmcr = (*HRTIMx).HRTIM_COMMON.BMCR;
    /* Enable/Disable the half mode */
    HRTIM_mcr &= !(0x20i32 as uint32_t);
    HRTIM_mcr |= (*pTimerInit).HalfModeEnable;
    /* Enable/Disable the timer start upon synchronization event reception */
    HRTIM_mcr &= !(0x800i32 as uint32_t);
    HRTIM_mcr |= (*pTimerInit).StartOnSync;
    /* Enable/Disable the timer reset upon synchronization event reception */
    HRTIM_mcr &= !(0x400i32 as uint32_t);
    HRTIM_mcr |= (*pTimerInit).ResetOnSync;
    /* Enable/Disable the DAC synchronization event generation */
    HRTIM_mcr &= !(0x6000000i32 as uint32_t);
    HRTIM_mcr |= (*pTimerInit).DACSynchro;
    /* Enable/Disable preload mechanism for timer registers */
    HRTIM_mcr &= !(0x8000000i32 as uint32_t);
    HRTIM_mcr |= (*pTimerInit).PreloadEnable;
    /* Master timer registers update handling */
    HRTIM_mcr &= !0xc0000000u32;
    HRTIM_mcr |= (*pTimerInit).UpdateGating << 2i32;
    /* Enable/Disable registers update on repetition */
    HRTIM_mcr &= !(0x20000000i32 as uint32_t);
    HRTIM_mcr |= (*pTimerInit).RepetitionUpdate;
    /* Set the timer burst mode */
    HRTIM_bmcr &= !(0x10000i32 as uint32_t);
    HRTIM_bmcr |= (*pTimerInit).BurstMode;
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCR as
                                    *mut uint32_t, HRTIM_mcr);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMCR as
                                    *mut uint32_t, HRTIM_bmcr);
}
/* *
  * @brief  Configures timing unit (timer A to timer E) in waveform mode 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  pTimerInit: pointer to the timer initialization data structure
  * @retval None
  */
unsafe extern "C" fn HRTIM_TimingUnitWaveform_Config(mut HRTIMx:
                                                         *mut HRTIM_TypeDef,
                                                     mut TimerIdx: uint32_t,
                                                     mut pTimerInit:
                                                         *mut HRTIM_TimerInitTypeDef) {
    let mut HRTIM_timcr: uint32_t = 0;
    let mut HRTIM_bmcr: uint32_t = 0;
    /* Configure timing unit */
    HRTIM_timcr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].TIMxCR;
    HRTIM_bmcr = (*HRTIMx).HRTIM_COMMON.BMCR;
    /* Enable/Disable the half mode */
    HRTIM_timcr &= !(0x20i32 as uint32_t);
    HRTIM_timcr |= (*pTimerInit).HalfModeEnable;
    /* Enable/Disable the timer start upon synchronization event reception */
    HRTIM_timcr &= !(0x800i32 as uint32_t);
    HRTIM_timcr |= (*pTimerInit).StartOnSync;
    /* Enable/Disable the timer reset upon synchronization event reception */
    HRTIM_timcr &= !(0x400i32 as uint32_t);
    HRTIM_timcr |= (*pTimerInit).ResetOnSync;
    /* Enable/Disable the DAC synchronization event generation */
    HRTIM_timcr &= !(0x6000000i32 as uint32_t);
    HRTIM_timcr |= (*pTimerInit).DACSynchro;
    /* Enable/Disable preload mechanism for timer registers */
    HRTIM_timcr &= !(0x8000000i32 as uint32_t);
    HRTIM_timcr |= (*pTimerInit).PreloadEnable;
    /* Timing unit registers update handling */
    HRTIM_timcr &= !0xf0000000u32;
    HRTIM_timcr |= (*pTimerInit).UpdateGating;
    /* Enable/Disable registers update on repetition */
    HRTIM_timcr &= !(0x20000i32 as uint32_t);
    if (*pTimerInit).RepetitionUpdate == 0x20000000i32 as uint32_t {
        HRTIM_timcr |= 0x20000i32 as uint32_t
    }
    /* Set the timer burst mode */
    match TimerIdx {
        0 => {
            HRTIM_bmcr &= !(0x20000i32 as uint32_t);
            HRTIM_bmcr |= (*pTimerInit).BurstMode << 1i32
        }
        1 => {
            HRTIM_bmcr &= !(0x40000i32 as uint32_t);
            HRTIM_bmcr |= (*pTimerInit).BurstMode << 2i32
        }
        2 => {
            HRTIM_bmcr &= !(0x80000i32 as uint32_t);
            HRTIM_bmcr |= (*pTimerInit).BurstMode << 3i32
        }
        3 => {
            HRTIM_bmcr &= !(0x100000i32 as uint32_t);
            HRTIM_bmcr |= (*pTimerInit).BurstMode << 4i32
        }
        4 => {
            HRTIM_bmcr &= !(0x200000i32 as uint32_t);
            HRTIM_bmcr |= (*pTimerInit).BurstMode << 5i32
        }
        _ => { }
    }
    /* Update the HRTIMx registers */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].TIMxCR
                                    as *mut uint32_t, HRTIM_timcr);
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.BMCR as
                                    *mut uint32_t, HRTIM_bmcr);
}
/* *
  * @brief  Configures a compare unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  CompareUnit: Compare unit identifier
  * @param  pCompareCfg: pointer to the compare unit configuration data structure
  * @retval None
  */
unsafe extern "C" fn HRTIM_CompareUnitConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut CompareUnit: uint32_t,
                                             mut pCompareCfg:
                                                 *mut HRTIM_CompareCfgTypeDef) {
    if TimerIdx == 0x5i32 as uint32_t {
        /* Configure the compare unit of the master timer */
        match CompareUnit {
            1 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP1R
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            2 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP2R
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            4 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP3R
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            8 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_MASTER.MCMP4R
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            _ => { }
        }
    } else {
        /* Configure the compare unit of the timing unit */
        match CompareUnit {
            1 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].CMP1xR
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            2 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].CMP2xR
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            4 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].CMP3xR
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            8 => {
                ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                            as
                                                                            usize].CMP4xR
                                                as *mut uint32_t,
                                            (*pCompareCfg).CompareValue)
            }
            _ => { }
        }
    };
}
/* *
  * @brief  Configures a capture unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  CaptureUnit: Capture unit identifier
  * @param  pCaptureCfg: pointer to the compare unit configuration data structure
  * @retval None
  */
unsafe extern "C" fn HRTIM_CaptureUnitConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                             mut TimerIdx: uint32_t,
                                             mut CaptureUnit: uint32_t,
                                             mut Event: uint32_t) {
    let mut CaptureTrigger: uint32_t = 0x4i32 as uint32_t;
    match Event {
        1 => { CaptureTrigger = 0x4i32 as uint32_t }
        2 => { CaptureTrigger = 0x8i32 as uint32_t }
        4 => { CaptureTrigger = 0x10i32 as uint32_t }
        8 => { CaptureTrigger = 0x20i32 as uint32_t }
        16 => { CaptureTrigger = 0x40i32 as uint32_t }
        32 => { CaptureTrigger = 0x80i32 as uint32_t }
        64 => { CaptureTrigger = 0x100i32 as uint32_t }
        128 => { CaptureTrigger = 0x200i32 as uint32_t }
        256 => { CaptureTrigger = 0x400i32 as uint32_t }
        512 => { CaptureTrigger = 0x800i32 as uint32_t }
        _ => { }
    }
    match CaptureUnit {
        1 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT1xCR
                                            as *mut uint32_t, CaptureTrigger)
        }
        2 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].CPT2xCR
                                            as *mut uint32_t, CaptureTrigger)
        }
        _ => { }
    };
}
/* *
  * @brief  Configures the output of a timing unit 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  Output: timing unit output identifier
  * @param  pOutputCfg: pointer to the output configuration data structure
  * @retval None
  */
unsafe extern "C" fn HRTIM_OutputConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                        mut TimerIdx: uint32_t,
                                        mut Output: uint32_t,
                                        mut pOutputCfg:
                                            *mut HRTIM_OutputCfgTypeDef) {
    let mut HRTIM_outr: uint32_t = 0;
    let mut shift: uint32_t = 0i32 as uint32_t;
    HRTIM_outr = (*HRTIMx).HRTIM_TIMERx[TimerIdx as usize].OUTxR;
    match Output {
        1 | 4 | 16 | 64 | 256 => {
            /* Set the output set/reset crossbar */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].SETx1R
                                            as *mut uint32_t,
                                        (*pOutputCfg).SetSource);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTx1R
                                            as *mut uint32_t,
                                        (*pOutputCfg).ResetSource);
            shift = 0i32 as uint32_t
        }
        2 | 8 | 32 | 128 | 512 => {
            /* Set the output set/reset crossbar */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].SETx2R
                                            as *mut uint32_t,
                                        (*pOutputCfg).SetSource);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTx2R
                                            as *mut uint32_t,
                                        (*pOutputCfg).ResetSource);
            shift = 16i32 as uint32_t
        }
        _ => { }
    }
    /* Clear output config */
    HRTIM_outr &=
        !((0x2i32 as uint32_t | 0x4i32 as uint32_t | 0x8i32 as uint32_t |
               0x30i32 as uint32_t | 0x40i32 as uint32_t |
               0x80i32 as uint32_t) << shift);
    /* Set the polarity */
    HRTIM_outr |= (*pOutputCfg).Polarity << shift;
    /* Set the IDLE mode */
    HRTIM_outr |= (*pOutputCfg).IdleMode << shift;
    /* Set the IDLE state */
    HRTIM_outr |= (*pOutputCfg).IdleState << shift;
    /* Set the FAULT state */
    HRTIM_outr |= (*pOutputCfg).FaultState << shift;
    /* Set the chopper mode */
    HRTIM_outr |= (*pOutputCfg).ChopperModeEnable << shift;
    /* Set the burst mode entry mode */
    HRTIM_outr |= (*pOutputCfg).BurstModeEntryDelayed << shift;
    /* Update HRTIMx register */
    ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx as
                                                                usize].OUTxR
                                    as *mut uint32_t, HRTIM_outr);
}
/* *
  * @brief  Configures an external event channel 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  Event: Event channel identifier
  * @param  pEventCfg: pointer to the event channel configuration data structure
  * @retval None
  */
unsafe extern "C" fn HRTIM_ExternalEventConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                               mut Event: uint32_t,
                                               mut pEventCfg:
                                                   *mut HRTIM_EventCfgTypeDef) {
    let mut hrtim_eecr1: uint32_t = 0;
    let mut hrtim_eecr2: uint32_t = 0;
    let mut hrtim_eecr3: uint32_t = 0;
    /* Configure external event channel */
    hrtim_eecr1 = (*HRTIMx).HRTIM_COMMON.EECR1;
    hrtim_eecr2 = (*HRTIMx).HRTIM_COMMON.EECR2;
    hrtim_eecr3 = (*HRTIMx).HRTIM_COMMON.EECR3;
    match Event {
        1 => {
            hrtim_eecr1 &=
                !(0x3i32 as uint32_t | 0x4i32 as uint32_t |
                      0x18i32 as uint32_t | 0x20i32 as uint32_t);
            hrtim_eecr1 |= (*pEventCfg).Source;
            hrtim_eecr1 |= (*pEventCfg).Polarity;
            hrtim_eecr1 |= (*pEventCfg).Sensitivity;
            /* Update the HRTIM registers (all bit fields but EE1FAST bit) */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1);
            /* Update the HRTIM registers (EE1FAST bit) */
            hrtim_eecr1 |= (*pEventCfg).FastMode;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1)
        }
        2 => {
            hrtim_eecr1 &=
                !(0xc0i32 as uint32_t | 0x100i32 as uint32_t |
                      0x600i32 as uint32_t | 0x800i32 as uint32_t);
            hrtim_eecr1 |= (*pEventCfg).Source << 6i32;
            hrtim_eecr1 |= (*pEventCfg).Polarity << 6i32;
            hrtim_eecr1 |= (*pEventCfg).Sensitivity << 6i32;
            /* Update the HRTIM registers (all bit fields but EE2FAST bit) */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1);
            /* Update the HRTIM registers (EE2FAST bit) */
            hrtim_eecr1 |= (*pEventCfg).FastMode << 6i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1)
        }
        4 => {
            hrtim_eecr1 &=
                !(0x3000i32 as uint32_t | 0x4000i32 as uint32_t |
                      0x18000i32 as uint32_t | 0x20000i32 as uint32_t);
            hrtim_eecr1 |= (*pEventCfg).Source << 12i32;
            hrtim_eecr1 |= (*pEventCfg).Polarity << 12i32;
            hrtim_eecr1 |= (*pEventCfg).Sensitivity << 12i32;
            /* Update the HRTIM registers (all bit fields but EE3FAST bit) */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1);
            /* Update the HRTIM registers (EE3FAST bit) */
            hrtim_eecr1 |= (*pEventCfg).FastMode << 12i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1)
        }
        8 => {
            hrtim_eecr1 &=
                !(0xc0000i32 as uint32_t | 0x100000i32 as uint32_t |
                      0x600000i32 as uint32_t | 0x800000i32 as uint32_t);
            hrtim_eecr1 |= (*pEventCfg).Source << 18i32;
            hrtim_eecr1 |= (*pEventCfg).Polarity << 18i32;
            hrtim_eecr1 |= (*pEventCfg).Sensitivity << 18i32;
            /* Update the HRTIM registers (all bit fields but EE4FAST bit) */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1);
            /* Update the HRTIM registers (EE4FAST bit) */
            hrtim_eecr1 |= (*pEventCfg).FastMode << 18i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1)
        }
        16 => {
            hrtim_eecr1 &=
                !(0x3000000i32 as uint32_t | 0x4000000i32 as uint32_t |
                      0x18000000i32 as uint32_t | 0x20000000i32 as uint32_t);
            hrtim_eecr1 |= (*pEventCfg).Source << 24i32;
            hrtim_eecr1 |= (*pEventCfg).Polarity << 24i32;
            hrtim_eecr1 |= (*pEventCfg).Sensitivity << 24i32;
            /* Update the HRTIM registers (all bit fields but EE5FAST bit) */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1);
            /* Update the HRTIM registers (EE5FAST bit) */
            hrtim_eecr1 |= (*pEventCfg).FastMode << 24i32;
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR1 as
                                            *mut uint32_t, hrtim_eecr1)
        }
        32 => {
            hrtim_eecr2 &=
                !(0x3i32 as uint32_t | 0x4i32 as uint32_t |
                      0x18i32 as uint32_t);
            hrtim_eecr2 |= (*pEventCfg).Source;
            hrtim_eecr2 |= (*pEventCfg).Polarity;
            hrtim_eecr2 |= (*pEventCfg).Sensitivity;
            hrtim_eecr3 &= !(0xfi32 as uint32_t);
            hrtim_eecr3 |= (*pEventCfg).Filter;
            /* Update the HRTIM registers */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR2 as
                                            *mut uint32_t, hrtim_eecr2);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR3 as
                                            *mut uint32_t, hrtim_eecr3)
        }
        64 => {
            hrtim_eecr2 &=
                !(0xc0i32 as uint32_t | 0x100i32 as uint32_t |
                      0x600i32 as uint32_t);
            hrtim_eecr2 |= (*pEventCfg).Source << 6i32;
            hrtim_eecr2 |= (*pEventCfg).Polarity << 6i32;
            hrtim_eecr2 |= (*pEventCfg).Sensitivity << 6i32;
            hrtim_eecr3 &= !(0x3c0i32 as uint32_t);
            hrtim_eecr3 |= (*pEventCfg).Filter << 6i32;
            /* Update the HRTIM registers */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR2 as
                                            *mut uint32_t, hrtim_eecr2);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR3 as
                                            *mut uint32_t, hrtim_eecr3)
        }
        128 => {
            hrtim_eecr2 &=
                !(0x3000i32 as uint32_t | 0x4000i32 as uint32_t |
                      0x18000i32 as uint32_t);
            hrtim_eecr2 |= (*pEventCfg).Source << 12i32;
            hrtim_eecr2 |= (*pEventCfg).Polarity << 12i32;
            hrtim_eecr2 |= (*pEventCfg).Sensitivity << 12i32;
            hrtim_eecr3 &= !(0xf000i32 as uint32_t);
            hrtim_eecr3 |= (*pEventCfg).Filter << 12i32;
            /* Update the HRTIM registers */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR2 as
                                            *mut uint32_t, hrtim_eecr2);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR3 as
                                            *mut uint32_t, hrtim_eecr3)
        }
        256 => {
            hrtim_eecr2 &=
                !(0xc0000i32 as uint32_t | 0x100000i32 as uint32_t |
                      0x600000i32 as uint32_t);
            hrtim_eecr2 |= (*pEventCfg).Source << 18i32;
            hrtim_eecr2 |= (*pEventCfg).Polarity << 18i32;
            hrtim_eecr2 |= (*pEventCfg).Sensitivity << 18i32;
            hrtim_eecr3 &= !(0x3c0000i32 as uint32_t);
            hrtim_eecr3 |= (*pEventCfg).Filter << 18i32;
            /* Update the HRTIM registers */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR2 as
                                            *mut uint32_t, hrtim_eecr2);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR3 as
                                            *mut uint32_t, hrtim_eecr3)
        }
        512 => {
            hrtim_eecr2 &=
                !(0x3000000i32 as uint32_t | 0x4000000i32 as uint32_t |
                      0x18000000i32 as uint32_t);
            hrtim_eecr2 |= (*pEventCfg).Source << 24i32;
            hrtim_eecr2 |= (*pEventCfg).Polarity << 24i32;
            hrtim_eecr2 |= (*pEventCfg).Sensitivity << 24i32;
            hrtim_eecr3 &= !(0xf000000i32 as uint32_t);
            hrtim_eecr3 |= (*pEventCfg).Filter << 24i32;
            /* Update the HRTIM registers */
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR2 as
                                            *mut uint32_t, hrtim_eecr2);
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_COMMON.EECR3 as
                                            *mut uint32_t, hrtim_eecr3)
        }
        _ => { }
    };
}
/* *
  * @brief  Configures the timer counter reset 
  * @param  HRTIMx: pointer to HRTIMx peripheral
  * @param  TimerIdx: Timer index
  * @param  Event: Event channel identifier
  * @retval None
  */
unsafe extern "C" fn HRTIM_TIM_ResetConfig(mut HRTIMx: *mut HRTIM_TypeDef,
                                           mut TimerIdx: uint32_t,
                                           mut Event: uint32_t) {
    match Event {
        1 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x200i32 as uint32_t)
        }
        2 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x400i32 as uint32_t)
        }
        4 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x800i32 as uint32_t)
        }
        8 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x1000i32 as uint32_t)
        }
        16 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x2000i32 as uint32_t)
        }
        32 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x4000i32 as uint32_t)
        }
        64 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x8000i32 as uint32_t)
        }
        128 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x10000i32 as uint32_t)
        }
        256 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x20000i32 as uint32_t)
        }
        512 => {
            ::core::ptr::write_volatile(&mut (*HRTIMx).HRTIM_TIMERx[TimerIdx
                                                                        as
                                                                        usize].RSTxR
                                            as *mut uint32_t,
                                        0x40000i32 as uint32_t)
        }
        _ => { }
    };
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
