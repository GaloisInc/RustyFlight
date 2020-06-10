use ::libc;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct RTC_TypeDef {
    pub CRH: uint16_t,
    pub RESERVED0: uint16_t,
    pub CRL: uint16_t,
    pub RESERVED1: uint16_t,
    pub PRLH: uint16_t,
    pub RESERVED2: uint16_t,
    pub PRLL: uint16_t,
    pub RESERVED3: uint16_t,
    pub DIVH: uint16_t,
    pub RESERVED4: uint16_t,
    pub DIVL: uint16_t,
    pub RESERVED5: uint16_t,
    pub CNTH: uint16_t,
    pub RESERVED6: uint16_t,
    pub CNTL: uint16_t,
    pub RESERVED7: uint16_t,
    pub ALRH: uint16_t,
    pub RESERVED8: uint16_t,
    pub ALRL: uint16_t,
    pub RESERVED9: uint16_t,
}
/* !< RTC Prescaler MSB Mask */
/* *
  * @}
  */
/* * @defgroup RTC_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Private_Functions
  * @{
  */
/* *
  * @brief  Enables or disables the specified RTC interrupts.
  * @param  RTC_IT: specifies the RTC interrupts sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @param  NewState: new state of the specified RTC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ITConfig(mut RTC_IT: uint16_t,
                                      mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh0 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x2800 as libc::c_int as
                                               libc::c_uint) as
                   *mut RTC_TypeDef)).CRH;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint16_t>(fresh0
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int |
                                         RTC_IT as libc::c_int) as uint16_t as
                                        uint16_t)
    } else {
        let ref mut fresh1 =
            (*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x2800 as libc::c_int as
                                               libc::c_uint) as
                   *mut RTC_TypeDef)).CRH;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint16_t>(fresh1
                                                                                as
                                                                                *const uint16_t)
                                         as libc::c_int &
                                         !(RTC_IT as libc::c_int) as uint16_t
                                             as libc::c_int) as uint16_t as
                                        uint16_t)
    };
}
/* *
  * @brief  Enters the RTC configuration mode.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_EnterConfigMode() {
    /* Set the CNF flag to enter in the Configuration Mode */
    let ref mut fresh2 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).CRL;
    ::core::ptr::write_volatile(fresh2,
                                (::core::ptr::read_volatile::<uint16_t>(fresh2
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int |
                                     0x10 as libc::c_int as uint8_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Exits from the RTC configuration mode.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ExitConfigMode() {
    /* Reset the CNF flag to exit from the Configuration Mode */
    let ref mut fresh3 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).CRL;
    ::core::ptr::write_volatile(fresh3,
                                (::core::ptr::read_volatile::<uint16_t>(fresh3
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x10 as libc::c_int as uint8_t as
                                           uint16_t as libc::c_int) as
                                         uint16_t as libc::c_int) as uint16_t
                                    as uint16_t);
}
/* *
  * @brief  Gets the RTC counter value.
  * @param  None
  * @retval RTC counter value.
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetCounter() -> uint32_t {
    let mut tmp: uint16_t = 0 as libc::c_int as uint16_t;
    tmp =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).CNTL;
    return ((*((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x2800 as libc::c_int as
                                               libc::c_uint) as
                   *mut RTC_TypeDef)).CNTH as uint32_t) << 16 as libc::c_int |
               tmp as libc::c_uint;
}
/* *
  * @brief  Sets the RTC counter value.
  * @param  CounterValue: RTC counter new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetCounter(mut CounterValue: uint32_t) {
    RTC_EnterConfigMode();
    /* Set RTC COUNTER MSB word */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2800 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).CNTH as
                                    *mut uint16_t,
                                (CounterValue >> 16 as libc::c_int) as
                                    uint16_t);
    /* Set RTC COUNTER LSB word */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2800 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).CNTL as
                                    *mut uint16_t,
                                (CounterValue &
                                     0xffff as libc::c_int as uint32_t) as
                                    uint16_t);
    RTC_ExitConfigMode();
}
/* *
  * @brief  Sets the RTC prescaler value.
  * @param  PrescalerValue: RTC prescaler new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetPrescaler(mut PrescalerValue: uint32_t) {
    /* Check the parameters */
    RTC_EnterConfigMode();
    /* Set RTC PRESCALER MSB word */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2800 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).PRLH as
                                    *mut uint16_t,
                                ((PrescalerValue &
                                      0xf0000 as libc::c_int as uint32_t) >>
                                     16 as libc::c_int) as uint16_t);
    /* Set RTC PRESCALER LSB word */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2800 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).PRLL as
                                    *mut uint16_t,
                                (PrescalerValue &
                                     0xffff as libc::c_int as uint32_t) as
                                    uint16_t);
    RTC_ExitConfigMode();
}
/* *
  * @brief  Sets the RTC alarm value.
  * @param  AlarmValue: RTC alarm new value.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_SetAlarm(mut AlarmValue: uint32_t) {
    RTC_EnterConfigMode();
    /* Set the ALARM MSB word */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2800 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).ALRH as
                                    *mut uint16_t,
                                (AlarmValue >> 16 as libc::c_int) as
                                    uint16_t);
    /* Set the ALARM LSB word */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2800 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut RTC_TypeDef)).ALRL as
                                    *mut uint16_t,
                                (AlarmValue &
                                     0xffff as libc::c_int as uint32_t) as
                                    uint16_t);
    RTC_ExitConfigMode();
}
/* *
  * @brief  Gets the RTC divider value.
  * @param  None
  * @retval RTC Divider value.
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetDivider() -> uint32_t {
    let mut tmp: uint32_t = 0 as libc::c_int as uint32_t;
    tmp =
        ((*((0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
                as *mut RTC_TypeDef)).DIVH as uint32_t &
             0xf as libc::c_int as uint32_t) << 16 as libc::c_int;
    tmp |=
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).DIVL as libc::c_uint;
    return tmp;
}
/* *
  * @brief  Waits until last write operation on RTC registers has finished.
  * @note   This function must be called before any write to RTC registers.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WaitForLastTask() {
    /* Loop until RTOFF flag is set */
    while (*((0x40000000 as libc::c_int as
                  uint32_t).wrapping_add(0x2800 as libc::c_int as
                                             libc::c_uint) as
                 *mut RTC_TypeDef)).CRL as libc::c_int &
              0x20 as libc::c_int as uint16_t as libc::c_int ==
              RESET as libc::c_int as uint16_t as libc::c_int {
    };
}
/* *
  * @brief  Waits until the RTC registers (RTC_CNT, RTC_ALR and RTC_PRL)
  *   are synchronized with RTC APB clock.
  * @note   This function must be called before any read operation after an APB reset
  *   or an APB clock stop.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_WaitForSynchro() {
    /* Clear RSF flag */
    let ref mut fresh4 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).CRL;
    ::core::ptr::write_volatile(fresh4,
                                (::core::ptr::read_volatile::<uint16_t>(fresh4
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(0x8 as libc::c_int as uint16_t as
                                           libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
    /* Loop until RSF flag is set */
    while (*((0x40000000 as libc::c_int as
                  uint32_t).wrapping_add(0x2800 as libc::c_int as
                                             libc::c_uint) as
                 *mut RTC_TypeDef)).CRL as libc::c_int &
              0x8 as libc::c_int as uint16_t as libc::c_int ==
              RESET as libc::c_int as uint16_t as libc::c_int {
    };
}
/* *
  * @brief  Checks whether the specified RTC flag is set or not.
  * @param  RTC_FLAG: specifies the flag to check.
  *   This parameter can be one the following values:
  *     @arg RTC_FLAG_RTOFF: RTC Operation OFF flag
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag
  *     @arg RTC_FLAG_OW: Overflow flag
  *     @arg RTC_FLAG_ALR: Alarm flag
  *     @arg RTC_FLAG_SEC: Second flag
  * @retval The new state of RTC_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetFlagStatus(mut RTC_FLAG: uint16_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
              as *mut RTC_TypeDef)).CRL as libc::c_int &
           RTC_FLAG as libc::c_int !=
           RESET as libc::c_int as uint16_t as libc::c_int {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  * @brief  Clears the RTC's pending flags.
  * @param  RTC_FLAG: specifies the flag to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_FLAG_RSF: Registers Synchronized flag. This flag is cleared only after
  *                        an APB reset or an APB Clock stop.
  *     @arg RTC_FLAG_OW: Overflow flag
  *     @arg RTC_FLAG_ALR: Alarm flag
  *     @arg RTC_FLAG_SEC: Second flag
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ClearFlag(mut RTC_FLAG: uint16_t) {
    /* Check the parameters */
    /* Clear the corresponding RTC flag */
    let ref mut fresh5 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).CRL;
    ::core::ptr::write_volatile(fresh5,
                                (::core::ptr::read_volatile::<uint16_t>(fresh5
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(RTC_FLAG as libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
}
/* *
  * @brief  Checks whether the specified RTC interrupt has occurred or not.
  * @param  RTC_IT: specifies the RTC interrupts sources to check.
  *   This parameter can be one of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @retval The new state of the RTC_IT (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_GetITStatus(mut RTC_IT: uint16_t) -> ITStatus {
    let mut bitstatus: ITStatus = RESET;
    /* Check the parameters */
    bitstatus =
        ((*((0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
                as *mut RTC_TypeDef)).CRL as libc::c_int &
             RTC_IT as libc::c_int) as ITStatus;
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
              as *mut RTC_TypeDef)).CRH as libc::c_int & RTC_IT as libc::c_int
           != RESET as libc::c_int as uint16_t as libc::c_int &&
           bitstatus as libc::c_uint !=
               RESET as libc::c_int as uint16_t as libc::c_uint {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_rtc.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the RTC firmware 
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
/* * @addtogroup RTC
  * @{
  */
/* * @defgroup RTC_Exported_Types
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Exported_Constants
  * @{
  */
/* * @defgroup RTC_interrupts_define 
  * @{
  */
/* !< Overflow interrupt */
/* !< Alarm interrupt */
/* !< Second interrupt */
/* *
  * @}
  */
/* * @defgroup RTC_interrupts_flags 
  * @{
  */
/* !< RTC Operation OFF flag */
/* !< Registers Synchronized flag */
/* !< Overflow flag */
/* !< Alarm flag */
/* !< Second flag */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup RTC_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup RTC_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears the RTC's interrupt pending bits.
  * @param  RTC_IT: specifies the interrupt pending bit to clear.
  *   This parameter can be any combination of the following values:
  *     @arg RTC_IT_OW: Overflow interrupt
  *     @arg RTC_IT_ALR: Alarm interrupt
  *     @arg RTC_IT_SEC: Second interrupt
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn RTC_ClearITPendingBit(mut RTC_IT: uint16_t) {
    /* Check the parameters */
    /* Clear the corresponding RTC pending bit */
    let ref mut fresh6 =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2800 as libc::c_int as libc::c_uint)
               as *mut RTC_TypeDef)).CRL;
    ::core::ptr::write_volatile(fresh6,
                                (::core::ptr::read_volatile::<uint16_t>(fresh6
                                                                            as
                                                                            *const uint16_t)
                                     as libc::c_int &
                                     !(RTC_IT as libc::c_int) as uint16_t as
                                         libc::c_int) as uint16_t as
                                    uint16_t);
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
