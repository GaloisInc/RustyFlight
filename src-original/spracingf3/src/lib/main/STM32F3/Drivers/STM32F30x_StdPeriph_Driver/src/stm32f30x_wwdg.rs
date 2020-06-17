use core;
use libc;
extern "C" {
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
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct WWDG_TypeDef {
    pub CR: uint32_t,
    pub CFR: uint32_t,
    pub SR: uint32_t,
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup WWDG_Private_Functions
  * @{
  */
/* * @defgroup WWDG_Group1 Prescaler, Refresh window and Counter configuration functions
 *  @brief   Prescaler, Refresh window and Counter configuration functions 
 *
@verbatim   
  ==============================================================================
    ##### Prescaler, Refresh window and Counter configuration functions #####
  ==============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Deinitializes the WWDG peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_DeInit() {
    RCC_APB1PeriphResetCmd(0x800i32 as uint32_t, ENABLE);
    RCC_APB1PeriphResetCmd(0x800i32 as uint32_t, DISABLE);
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
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Clear WDGTB[1:0] bits */
    tmpreg =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2c00i32 as libc::c_uint)
               as *mut WWDG_TypeDef)).CFR & 0xfffffe7fu32;
    /* Set WDGTB[1:0] bits according to WWDG_Prescaler value */
    tmpreg |= WWDG_Prescaler;
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2c00i32
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
    let mut tmpreg: uint32_t = 0i32 as uint32_t;
    /* Check the parameters */
    /* Clear W[6:0] bits */
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (*((0x40000000i32 as
                                        uint32_t).wrapping_add(0x2c00i32 as
                                                                   libc::c_uint)
                                       as *mut WWDG_TypeDef)).CFR &
                                    0xffffff80u32);
    /* Set W[6:0] bits according to WindowValue value */
    ::core::ptr::write_volatile(&mut tmpreg as *mut uint32_t,
                                (::core::ptr::read_volatile::<uint32_t>(&tmpreg
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     WindowValue as libc::c_uint &
                                         0x7fi32 as uint8_t as uint32_t) as
                                    uint32_t as uint32_t);
    /* Store the new value */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2c00i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CFR as
                                    *mut uint32_t, tmpreg);
}
/* *
  * @brief  Enables the WWDG Early Wakeup interrupt(EWI).
  * @note   Once enabled this interrupt cannot be disabled except by a system reset. 
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_EnableIT() {
    let ref mut fresh0 =
        (*((0x40000000i32 as uint32_t).wrapping_add(0x2c00i32 as libc::c_uint)
               as *mut WWDG_TypeDef)).CFR;
    ::core::ptr::write_volatile(fresh0,
                                (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                            as
                                                                            *const uint32_t)
                                     as libc::c_uint |
                                     0x200i32 as uint16_t as libc::c_uint) as
                                    uint32_t as uint32_t);
}
/* *
  * @brief  Sets the WWDG counter value.
  * @param  Counter: specifies the watchdog counter value.
  *   This parameter must be a number between 0x40 and 0x7F (to prevent generating
  *   an immediate reset).  
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_SetCounter(mut Counter: uint8_t) {
    /* Check the parameters */
    /* Write to T[6:0] bits to configure the counter value, no need to do
     a read-modify-write; writing a 0 to WDGA bit does nothing */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2c00i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CR as
                                    *mut uint32_t,
                                (Counter as libc::c_int &
                                     0x7fi32 as uint8_t as libc::c_int) as
                                    uint32_t);
}
/* *
  * @}
  */
/* * @defgroup WWDG_Group2 WWDG activation functions
 *  @brief   WWDG activation functions 
 *
@verbatim   
  ==============================================================================
                    ##### WWDG activation function #####
  ==============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Enables WWDG and load the counter value.                  
  * @param  Counter: specifies the watchdog counter value.
  *   This parameter must be a number between 0x40 and 0x7F (to prevent generating
  *   an immediate reset).
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_Enable(mut Counter: uint8_t) {
    /* Check the parameters */
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2c00i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).CR as
                                    *mut uint32_t,
                                (0x80i32 as uint8_t as libc::c_int |
                                     Counter as libc::c_int) as uint32_t);
}
/* *
  * @}
  */
/* * @defgroup WWDG_Group3 Interrupts and flags management functions
 *  @brief   Interrupts and flags management functions 
 *
@verbatim   
  ==============================================================================
              ##### Interrupts and flags management functions #####
  ==============================================================================  

@endverbatim
  * @{
  */
/* *
  * @brief  Checks whether the Early Wakeup interrupt flag is set or not.
  * @param  None
  * @retval The new state of the Early Wakeup interrupt flag (SET or RESET).
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_GetFlagStatus() -> FlagStatus {
    let mut bitstatus: FlagStatus = RESET;
    if (*((0x40000000i32 as uint32_t).wrapping_add(0x2c00i32 as libc::c_uint)
              as *mut WWDG_TypeDef)).SR != RESET as libc::c_int as uint32_t {
        bitstatus = SET
    } else { bitstatus = RESET }
    return bitstatus;
}
/* *
  ******************************************************************************
  * @file    stm32f30x_wwdg.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the WWDG 
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
/* * @addtogroup WWDG
  * @{
  */ 
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
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
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/*  Function used to set the WWDG configuration to the default reset state ****/
/* Prescaler, Refresh window and Counter configuration functions **************/
/* WWDG activation functions **************************************************/
/* Interrupts and flags management functions **********************************/
/* *
  * @brief  Clears Early Wakeup interrupt flag.
  * @param  None
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn WWDG_ClearFlag() {
    ::core::ptr::write_volatile(&mut (*((0x40000000i32 as
                                             uint32_t).wrapping_add(0x2c00i32
                                                                        as
                                                                        libc::c_uint)
                                            as *mut WWDG_TypeDef)).SR as
                                    *mut uint32_t,
                                RESET as libc::c_int as uint32_t);
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
