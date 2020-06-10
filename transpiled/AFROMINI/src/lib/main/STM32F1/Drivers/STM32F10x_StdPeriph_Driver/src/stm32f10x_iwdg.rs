use ::libc;
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
/* * 
  * @brief Independent WATCHDOG
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct IWDG_TypeDef {
    pub KR: uint32_t,
    pub PR: uint32_t,
    pub RLR: uint32_t,
    pub SR: uint32_t,
}
/* *
  * @}
  */
/* * @defgroup IWDG_Private_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Private_Variables
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Private_FunctionPrototypes
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Private_Functions
  * @{
  */
/* *
  * @brief  Enables or disables write access to IWDG_PR and IWDG_RLR registers.
  * @param  IWDG_WriteAccess: new state of write access to IWDG_PR and IWDG_RLR registers.
  *   This parameter can be one of the following values:
  *     @arg IWDG_WriteAccess_Enable: Enable write access to IWDG_PR and IWDG_RLR registers
  *     @arg IWDG_WriteAccess_Disable: Disable write access to IWDG_PR and IWDG_RLR registers
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_WriteAccessCmd(mut IWDG_WriteAccess: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x3000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).KR as
                                    *mut uint32_t,
                                IWDG_WriteAccess as uint32_t);
}
/* *
  * @brief  Sets IWDG Prescaler value.
  * @param  IWDG_Prescaler: specifies the IWDG Prescaler value.
  *   This parameter can be one of the following values:
  *     @arg IWDG_Prescaler_4: IWDG prescaler set to 4
  *     @arg IWDG_Prescaler_8: IWDG prescaler set to 8
  *     @arg IWDG_Prescaler_16: IWDG prescaler set to 16
  *     @arg IWDG_Prescaler_32: IWDG prescaler set to 32
  *     @arg IWDG_Prescaler_64: IWDG prescaler set to 64
  *     @arg IWDG_Prescaler_128: IWDG prescaler set to 128
  *     @arg IWDG_Prescaler_256: IWDG prescaler set to 256
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_SetPrescaler(mut IWDG_Prescaler: uint8_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x3000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).PR as
                                    *mut uint32_t,
                                IWDG_Prescaler as uint32_t);
}
/* *
  * @brief  Sets IWDG Reload value.
  * @param  Reload: specifies the IWDG Reload value.
  *   This parameter must be a number between 0 and 0x0FFF.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_SetReload(mut Reload: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x3000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).RLR as
                                    *mut uint32_t, Reload as uint32_t);
}
/* *
  * @brief  Reloads IWDG counter with value defined in the reload register
  *   (write access to IWDG_PR and IWDG_RLR registers disabled).
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_ReloadCounter() {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x3000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).KR as
                                    *mut uint32_t,
                                0xaaaa as libc::c_int as uint16_t as
                                    uint32_t);
}
/* *
  * @brief  Enables IWDG (write access to IWDG_PR and IWDG_RLR registers disabled).
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_Enable() {
    ::core::ptr::write_volatile(&mut (*((0x40000000 as libc::c_int as
                                             uint32_t).wrapping_add(0x3000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).KR as
                                    *mut uint32_t,
                                0xcccc as libc::c_int as uint16_t as
                                    uint32_t);
}
/* *
  ******************************************************************************
  * @file    stm32f10x_iwdg.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the IWDG 
  *          firmware library.
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
/* * @addtogroup IWDG
  * @{
  */
/* * @defgroup IWDG_Exported_Types
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Exported_Constants
  * @{
  */
/* * @defgroup IWDG_WriteAccess
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_prescaler 
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Flag 
  * @{
  */
/* *
  * @}
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Exported_Macros
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_Exported_Functions
  * @{
  */
/* *
  * @brief  Checks whether the specified IWDG flag is set or not.
  * @param  IWDG_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg IWDG_FLAG_PVU: Prescaler Value Update on going
  *     @arg IWDG_FLAG_RVU: Reload Value Update on going
  * @retval The new state of IWDG_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_GetFlagStatus(mut IWDG_FLAG: uint16_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000 as libc::c_int as
               uint32_t).wrapping_add(0x3000 as libc::c_int as libc::c_uint)
              as *mut IWDG_TypeDef)).SR & IWDG_FLAG as libc::c_uint !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the flag status */
    return bitstatus;
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
