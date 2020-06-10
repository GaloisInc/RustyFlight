use ::libc;
extern "C" {
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                 GPIO_InitStruct: *mut GPIO_InitTypeDef);
    #[no_mangle]
    fn GPIO_SetBits(GPIOx: *mut GPIO_TypeDef, GPIO_Pin: uint16_t);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* * 
  * @brief General Purpose I/O
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_TypeDef {
    pub MODER: uint32_t,
    pub OTYPER: uint16_t,
    pub RESERVED0: uint16_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint16_t,
    pub RESERVED1: uint16_t,
    pub ODR: uint16_t,
    pub RESERVED2: uint16_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
    pub BRR: uint16_t,
    pub RESERVED3: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_gpio.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the GPIO 
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
/* * @addtogroup GPIO
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * @defgroup Configuration_Mode_enumeration 
  * @{
  */
pub type GPIOMode_TypeDef = libc::c_uint;
/* !< GPIO Analog In/Out Mode      */
/* !< GPIO Alternate function Mode */
pub const GPIO_Mode_AN: GPIOMode_TypeDef = 3;
/* !< GPIO Output Mode */
pub const GPIO_Mode_AF: GPIOMode_TypeDef = 2;
/* !< GPIO Input Mode */
pub const GPIO_Mode_OUT: GPIOMode_TypeDef = 1;
pub const GPIO_Mode_IN: GPIOMode_TypeDef = 0;
/* *
  * @}
  */
/* * @defgroup Output_type_enumeration
  * @{
  */
pub type GPIOOType_TypeDef = libc::c_uint;
pub const GPIO_OType_OD: GPIOOType_TypeDef = 1;
pub const GPIO_OType_PP: GPIOOType_TypeDef = 0;
/* *
  * @}
  */
/* * @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
pub type GPIOSpeed_TypeDef = libc::c_uint;
/* !< High Speed     */
/* !< Meduim Speed   */
pub const GPIO_Speed_Level_3: GPIOSpeed_TypeDef = 3;
/* !< Fast Speed     */
pub const GPIO_Speed_Level_2: GPIOSpeed_TypeDef = 2;
pub const GPIO_Speed_Level_1: GPIOSpeed_TypeDef = 1;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type GPIOPuPd_TypeDef = libc::c_uint;
pub const GPIO_PuPd_DOWN: GPIOPuPd_TypeDef = 2;
pub const GPIO_PuPd_UP: GPIOPuPd_TypeDef = 1;
pub const GPIO_PuPd_NOPULL: GPIOPuPd_TypeDef = 0;
/* *
  * @}
  */
/* * 
  * @brief  GPIO Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct GPIO_InitTypeDef {
    pub GPIO_Pin: uint32_t,
    pub GPIO_Mode: GPIOMode_TypeDef,
    pub GPIO_Speed: GPIOSpeed_TypeDef,
    pub GPIO_OType: GPIOOType_TypeDef,
    pub GPIO_PuPd: GPIOPuPd_TypeDef,
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub static mut hardwareRevision: uint8_t = 1 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn detectHardwareRevision() {
    let mut GPIO_InitStructure: GPIO_InitTypeDef =
        {
            let mut init =
                GPIO_InitTypeDef{GPIO_Pin: 0,
                                 GPIO_Mode: GPIO_Mode_OUT,
                                 GPIO_Speed: GPIO_Speed_Level_2,
                                 GPIO_OType: GPIO_OType_PP,
                                 GPIO_PuPd: GPIO_PuPd_NOPULL,};
            init
        };
    // GYRO CS as output
    GPIO_InitStructure.GPIO_Pin =
        (0x20 as libc::c_int as uint16_t as libc::c_int |
             0x1000 as libc::c_int as uint16_t as libc::c_int) as uint32_t;
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0x400
                                                                             as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_SetBits((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x400
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut GPIO_TypeDef,
                 GPIO_InitStructure.GPIO_Pin as uint16_t);
    GPIO_InitStructure.GPIO_Pin = 0x80 as libc::c_int as uint16_t as uint32_t;
    GPIO_Init((0x40000000 as libc::c_int as
                   uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                              libc::c_uint).wrapping_add(0 as
                                                                             libc::c_int
                                                                             as
                                                                             libc::c_uint)
                  as *mut GPIO_TypeDef, &mut GPIO_InitStructure);
    GPIO_SetBits((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x8000000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut GPIO_TypeDef,
                 GPIO_InitStructure.GPIO_Pin as uint16_t);
}
/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#[no_mangle]
pub unsafe extern "C" fn updateHardwareRevision() { }
