use core;
use libc;
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct IWDG_TypeDef {
    pub KR: uint32_t,
    pub PR: uint32_t,
    pub RLR: uint32_t,
    pub SR: uint32_t,
    pub WINR: uint32_t,
    /* !< IWDG Window register,    Address offset: 0x10 */
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup IWDG_Private_Functions
  * @{
  */
/* * @defgroup IWDG_Group1 Prescaler and Counter configuration functions
 *  @brief   Prescaler and Counter configuration functions
 *
@verbatim   
 ===============================================================================
            ##### Prescaler and Counter configuration functions #####
 ===============================================================================  

@endverbatim
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
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x3000i32
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
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x3000i32
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
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x3000i32
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
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x3000i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).KR as
                                    *mut uint32_t,
                                0xaaaai32 as uint16_t as uint32_t);
}
/* *
  * @brief  Sets the IWDG window value.
  * @param  WindowValue: specifies the window value to be compared to the downcounter.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_SetWindowValue(mut WindowValue: uint16_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x3000i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).WINR as
                                    *mut uint32_t, WindowValue as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup IWDG_Group2 IWDG activation function
 *  @brief   IWDG activation function 
 *
@verbatim   
 ===============================================================================
                    ##### IWDG activation function #####
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables IWDG (write access to IWDG_PR and IWDG_RLR registers disabled).
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_Enable() {
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x3000i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut IWDG_TypeDef)).KR as
                                    *mut uint32_t,
                                0xcccci32 as uint16_t as uint32_t);
}
/* *
  ******************************************************************************
  * @file    stm32f30x_iwdg.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the IWDG 
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
/* * @addtogroup IWDG
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
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
/* * @defgroup IWDG_Reload_Value
  * @{
  */
/* *
  * @}
  */
/* * @defgroup IWDG_CounterWindow_Value
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
/* Prescaler and Counter configuration functions ******************************/
/* IWDG activation function ***************************************************/
/* Flag management function ***************************************************/
/* *
  * @}
  */
/* * @defgroup IWDG_Group3 Flag management function 
 *  @brief  Flag management function  
 *
@verbatim   
 ===============================================================================
                     ##### Flag management function ##### 
 ===============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Checks whether the specified IWDG flag is set or not.
  * @param  IWDG_FLAG: specifies the flag to check.
  *   This parameter can be one of the following values:
  *     @arg IWDG_FLAG_PVU: Prescaler Value Update on going
  *     @arg IWDG_FLAG_RVU: Reload Value Update on going
  *     @arg IWDG_FLAG_WVU: Counter Window Value Update on going
  * @retval The new state of IWDG_FLAG (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn IWDG_GetFlagStatus(mut IWDG_FLAG: uint16_t)
 -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    /* Check the parameters */
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x3000i32 as libc::c_uint)
              as *mut IWDG_TypeDef)).SR & IWDG_FLAG as libc::c_uint !=
           RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    /* Return the flag status */
    return bitstatus;
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
