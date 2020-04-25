use core;
use libc;
pub type __uint32_t = libc::c_uint;
pub type uint32_t = __uint32_t;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* * 
  * @brief Debug MCU
  */
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DBGMCU_TypeDef {
    pub IDCODE: uint32_t,
    pub CR: uint32_t,
    pub APB1FZ: uint32_t,
    pub APB2FZ: uint32_t,
    /* !< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* * @defgroup DBGMCU_Private_Functions
  * @{
  */
/* * @defgroup DBGMCU_Group1 Device and Revision ID management functions
 *  @brief   Device and Revision ID management functions
 *
@verbatim
  ==============================================================================
            ##### Device and Revision ID management functions #####
  ==============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Returns the device revision identifier.
  * @param  None
  * @retval Device revision identifier
  */
#[no_mangle]
pub unsafe extern "C" fn DBGMCU_GetREVID() -> uint32_t {
    return (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).IDCODE >> 16i32;
}
/* *
  * @brief   Returns the device identifier.
  * @param  None
  * @retval Device identifier
  */
#[no_mangle]
pub unsafe extern "C" fn DBGMCU_GetDEVID() -> uint32_t {
    return (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).IDCODE &
               0xfffi32 as uint32_t;
}
/* *
  * @}
  */
/* * @defgroup DBGMCU_Group2 Peripherals Configuration functions
 *  @brief   Peripherals Configuration
 *
@verbatim
  ==============================================================================
               ##### Peripherals Configuration functions #####
  ==============================================================================

@endverbatim
  * @{
  */
/* *
  * @brief  Configures low power mode behavior when the MCU is in Debug mode.
  * @param  DBGMCU_Periph: specifies the low power mode.
  *   This parameter can be any combination of the following values:
  *     @arg DBGMCU_SLEEP: Keep debugger connection during SLEEP mode.              
  *     @arg DBGMCU_STOP: Keep debugger connection during STOP mode.               
  *     @arg DBGMCU_STANDBY: Keep debugger connection during STANDBY mode.        
  * @param  NewState: new state of the specified low power mode in Debug mode.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DBGMCU_Config(mut DBGMCU_Periph: uint32_t,
                                       mut NewState: FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh0 = (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh0,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh0
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | DBGMCU_Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh1 = (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).CR;
        ::core::ptr::write_volatile(fresh1,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh1
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !DBGMCU_Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  * @brief  Configures APB1 peripheral behavior when the MCU is in Debug mode.
  * @param  DBGMCU_Periph: specifies the APB1 peripheral.
  *   This parameter can be any combination of the following values:        
  *     @arg DBGMCU_TIM2_STOP: TIM2 counter stopped when Core is halted.          
  *     @arg DBGMCU_TIM3_STOP: TIM3 counter stopped when Core is halted.          
  *     @arg DBGMCU_TIM4_STOP: TIM4 counter stopped when Core is halted.
  *     @arg DBGMCU_TIM6_STOP: TIM6 counter stopped when Core is halted.          
  *     @arg DBGMCU_TIM7_STOP: TIM7 counter stopped when Core is halted.
  *     @arg DBGMCU_RTC_STOP: RTC Calendar and Wakeup counter are stopped when 
  *          Core is halted. 
  *     @arg DBGMCU_WWDG_STOP: Debug WWDG stopped when Core is halted.
  *     @arg DBGMCU_IWDG_STOP: Debug IWDG stopped when Core is halted.        
  *     @arg DBGMCU_I2C1_SMBUS_TIMEOUT: I2C1 SMBUS timeout mode stopped when 
  *          Core is halted.
  *     @arg DBGMCU_I2C2_SMBUS_TIMEOUT: I2C2 SMBUS timeout mode stopped when 
  *          Core is halted.
  *     @arg DBGMCU_CAN1_STOP: Debug CAN2 stopped when Core is halted.        
  * @param  NewState: new state of the specified APB1 peripheral in Debug mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DBGMCU_APB1PeriphConfig(mut DBGMCU_Periph: uint32_t,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh2 = (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).APB1FZ;
        ::core::ptr::write_volatile(fresh2,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh2
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | DBGMCU_Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh3 = (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).APB1FZ;
        ::core::ptr::write_volatile(fresh3,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh3
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !DBGMCU_Periph) as
                                        uint32_t as uint32_t)
    };
}
/* *
  ******************************************************************************
  * @file    stm32f30x_dbgmcu.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the DBGMCU firmware library.
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
/* * @addtogroup DBGMCU
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* * @defgroup DBGMCU_Exported_Constants
  * @{
  */
/* *
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
/* Device and Revision ID management functions ********************************/
/* Peripherals Configuration functions ****************************************/
/* *
  * @brief  Configures APB2 peripheral behavior when the MCU is in Debug mode.
  * @param  DBGMCU_Periph: specifies the APB2 peripheral.
  *   This parameter can be any combination of the following values:       
  *     @arg DBGMCU_TIM1_STOP: TIM1 counter stopped when Core is halted.   
  *     @arg DBGMCU_TIM8_STOP: TIM8 counter stopped when Core is halted. 
  *     @arg DBGMCU_TIM15_STOP: TIM15 counter stopped when Core is halted.                
  *     @arg DBGMCU_TIM16_STOP: TIM16 counter stopped when Core is halted.
  *     @arg DBGMCU_TIM17_STOP: TIM17 counter stopped when Core is halted.   
  * @param  NewState: new state of the specified APB2 peripheral in Debug mode.
  *         This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
#[no_mangle]
pub unsafe extern "C" fn DBGMCU_APB2PeriphConfig(mut DBGMCU_Periph: uint32_t,
                                                 mut NewState:
                                                     FunctionalState) {
    /* Check the parameters */
    if NewState as libc::c_uint != DISABLE as libc::c_int as libc::c_uint {
        let ref mut fresh4 = (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).APB2FZ;
        ::core::ptr::write_volatile(fresh4,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh4
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint | DBGMCU_Periph) as
                                        uint32_t as uint32_t)
    } else {
        let ref mut fresh5 = (*(0xe0042000u32 as *mut DBGMCU_TypeDef)).APB2FZ;
        ::core::ptr::write_volatile(fresh5,
                                    (::core::ptr::read_volatile::<uint32_t>(fresh5
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint & !DBGMCU_Periph) as
                                        uint32_t as uint32_t)
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
/* *
  * @}
  */
