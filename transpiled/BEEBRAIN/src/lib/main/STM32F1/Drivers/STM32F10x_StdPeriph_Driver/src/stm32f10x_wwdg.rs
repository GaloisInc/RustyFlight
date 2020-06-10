use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_APB1PeriphResetCmd(RCC_APB1Periph: uint32_t,
                              NewState: FunctionalState);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint32_t = __uint32_t;
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct WWDG_TypeDef {
    pub CR: uint32_t,
    pub CFR: uint32_t,
    pub SR: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup WWDG_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup WWDG_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup WWDG_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup WWDG_Private_Functions
  * @{
  */
/* *
  * @brief  Deinitializes the WWDG peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_DeInit() {
    RCC_APB1PeriphResetCmd(0x800 as libc::c_int as uint32_t, ENABLE);
    RCC_APB1PeriphResetCmd(0x800 as libc::c_int as uint32_t, DISABLE);
}
/* *
  * @brief  Sets the WWDG Prescaler.
  * @param  WWDG_Prescaler: specifies the WWDG Prescaler.
  *   This parameter can be one of the following values:
  *     @arg WWDG_Prescaler_1: WWDG counter clock = (PCLK1/4096)/1
  *     @arg WWDG_Prescaler_2: WWDG counter clock = (PCLK1/4096)/2
  *     @arg WWDG_Prescaler_4: WWDG counter clock = (PCLK1/4096)/4
  *     @arg WWDG_Prescaler_8: WWDG counter clock = (PCLK1/4096)/8
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_SetPrescaler(mut WWDG_Prescaler: uint32_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Clear WDGTB[1:0] bits */
    tmpreg =
        (*((0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x2c00 as libc::c_int as libc::c_uint)
               as *mut WWDG_TypeDef)).CFR & 0xfffffe7f as libc::c_uint;
    /* Set WDGTB[1:0] bits according to WWDG_Prescaler value */
    tmpreg |= WWDG_Prescaler;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CFR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Sets the WWDG window value.
  * @param  WindowValue: specifies the window value to be compared to the downcounter.
  *   This parameter value must be lower than 0x80.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_SetWindowValue(mut WindowValue: uint8_t) {
    let mut tmpreg: uint32_t = 0 as libc::c_int as uint32_t;
    /* Check the parameters */
    /* Clear W[6:0] bits */
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000 as libc::c_int as
                                        uint32_t).wrapping_add(0x2c00 as
                                                                   libc::c_int
                                                                   as
                                                                   libc::c_uint)
                                       as *mut WWDG_TypeDef)).CFR &
                                    0xffffff80 as libc::c_uint);
    /* Set W[6:0] bits according to WindowValue value */
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmpreg
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     WindowValue as libc::c_uint &
                                         0x7f as libc::c_int as uint8_t as
                                             uint32_t) as uint32_t as
                                    uint32_t);
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CFR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables the WWDG Early Wakeup interrupt(EWI).
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_EnableIT() {
    ::core::ptr::write_volatile((0x42000000 as libc::c_int as
                                     uint32_t).wrapping_add((0x40000000 as
                                                                 libc::c_int
                                                                 as
                                                                 uint32_t).wrapping_add(0x2c00
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
                                                                                                                                                                                     libc::c_uint)).wrapping_add((0x9
                                                                                                                                                                                                                      as
                                                                                                                                                                                                                      libc::c_int
                                                                                                                                                                                                                      *
                                                                                                                                                                                                                      4
                                                                                                                                                                                                                          as
                                                                                                                                                                                                                          libc::c_int)
                                                                                                                                                                                                                     as
                                                                                                                                                                                                                     libc::c_uint)
                                    as *mut uint32_t,
                                ENABLE as libc::c_int as uint32_t);
}
/* *
  * @brief  Sets the WWDG counter value.
  * @param  Counter: specifies the watchdog counter value.
  *   This parameter must be a number between 0x40 and 0x7F.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_SetCounter(mut Counter: uint8_t) {
    /* Check the parameters */
    /* Write to T[6:0] bits to configure the counter value, no need to do
     a read-modify-write; writing a 0 to WDGA bit does nothing */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CR as
                                    *mut uint32_t,
                                (Counter as libc::c_int &
                                     0x7f as libc::c_int as uint8_t as
                                         libc::c_int) as uint32_t);
}
/* *
  * @brief  Enables WWDG and load the counter value.                  
  * @param  Counter: specifies the watchdog counter value.
  *   This parameter must be a number between 0x40 and 0x7F.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_Enable(mut Counter: uint8_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CR as
                                    *mut uint32_t,
                                0x80 as libc::c_int as uint32_t |
                                    Counter as libc::c_uint);
}
/* *
  * @brief  Checks whether the Early Wakeup interrupt flag is set or not.
  * @param  None
  * @retval The new state of the Early Wakeup interrupt flag (SET or RESET)
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_GetFlagStatus() -> FlagStatus {
    return (*((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x2c00 as libc::c_int as
                                              libc::c_uint) as
                  *mut WWDG_TypeDef)).SR as FlagStatus;
}
/* *
  ******************************************************************************
  * @file    stm32f10x_wwdg.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the WWDG firmware
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
/* * @addtogroup WWDG
  * @{
  */
/* * @defgroup WWDG_Exported_Types
  * @{
  */
/* *
  * @}
  */
/* * @defgroup WWDG_Exported_Constants
  * @{
  */
/* * @defgroup WWDG_Prescaler 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup WWDG_Exported_Macros
  * @{
  */ 
/* *
  * @}
  */
/* * @defgroup WWDG_Exported_Functions
  * @{
  */
/* *
  * @brief  Clears Early Wakeup interrupt flag.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_ClearFlag() {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x2c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).SR as
                                    *mut uint32_t,
                                RESET as libc::c_int as uint32_t);
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
