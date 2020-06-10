use ::libc;
extern "C" {
    #[no_mangle]
    fn LL_GPIO_Init(GPIOx: *mut GPIO_TypeDef,
                    GPIO_InitStruct: *mut LL_GPIO_InitTypeDef) -> ErrorStatus;
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
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
    pub OTYPER: uint32_t,
    pub OSPEEDR: uint32_t,
    pub PUPDR: uint32_t,
    pub IDR: uint32_t,
    pub ODR: uint32_t,
    pub BSRR: uint32_t,
    pub LCKR: uint32_t,
    pub AFR: [uint32_t; 2],
}
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
pub type ErrorStatus = libc::c_uint;
pub const SUCCESS: ErrorStatus = 1;
pub const ERROR: ErrorStatus = 0;
pub type size_t = libc::c_ulong;
/* *
  ******************************************************************************
  * @file    stm32f7xx_ll_gpio.h
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    14-April-2017
  * @brief   Header file of GPIO LL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
/* * @addtogroup STM32F7xx_LL_Driver
  * @{
  */
/* * @defgroup GPIO_LL GPIO
  * @{
  */
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* * @defgroup GPIO_LL_Private_Macros GPIO Private Macros
  * @{
  */
/* *
  * @}
  */
/*USE_FULL_LL_DRIVER*/
/* Exported types ------------------------------------------------------------*/
/* * @defgroup GPIO_LL_ES_INIT GPIO Exported Init structures
  * @{
  */
/* *
  * @brief LL GPIO Init Structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct LL_GPIO_InitTypeDef {
    pub Pin: uint32_t,
    pub Mode: uint32_t,
    pub Speed: uint32_t,
    pub OutputType: uint32_t,
    pub Pull: uint32_t,
    pub Alternate: uint32_t,
}
pub type resourceOwner_e = libc::c_uint;
pub const OWNER_TOTAL_COUNT: resourceOwner_e = 55;
pub const OWNER_SPI_PREINIT_OPU: resourceOwner_e = 54;
pub const OWNER_SPI_PREINIT_IPU: resourceOwner_e = 53;
pub const OWNER_USB_MSC_PIN: resourceOwner_e = 52;
pub const OWNER_PINIO: resourceOwner_e = 51;
pub const OWNER_RX_SPI: resourceOwner_e = 50;
pub const OWNER_RANGEFINDER: resourceOwner_e = 49;
pub const OWNER_TIMUP: resourceOwner_e = 48;
pub const OWNER_CAMERA_CONTROL: resourceOwner_e = 47;
pub const OWNER_ESCSERIAL: resourceOwner_e = 46;
pub const OWNER_RX_BIND_PLUG: resourceOwner_e = 45;
pub const OWNER_COMPASS_CS: resourceOwner_e = 44;
pub const OWNER_VTX: resourceOwner_e = 43;
pub const OWNER_TRANSPONDER: resourceOwner_e = 42;
pub const OWNER_LED_STRIP: resourceOwner_e = 41;
pub const OWNER_INVERTER: resourceOwner_e = 40;
pub const OWNER_RX_BIND: resourceOwner_e = 39;
pub const OWNER_OSD: resourceOwner_e = 38;
pub const OWNER_BEEPER: resourceOwner_e = 37;
pub const OWNER_USB_DETECT: resourceOwner_e = 36;
pub const OWNER_USB: resourceOwner_e = 35;
pub const OWNER_COMPASS_EXTI: resourceOwner_e = 34;
pub const OWNER_BARO_EXTI: resourceOwner_e = 33;
pub const OWNER_MPU_EXTI: resourceOwner_e = 32;
pub const OWNER_SPI_CS: resourceOwner_e = 31;
pub const OWNER_RX_SPI_CS: resourceOwner_e = 30;
pub const OWNER_OSD_CS: resourceOwner_e = 29;
pub const OWNER_MPU_CS: resourceOwner_e = 28;
pub const OWNER_BARO_CS: resourceOwner_e = 27;
pub const OWNER_FLASH_CS: resourceOwner_e = 26;
pub const OWNER_SDCARD_DETECT: resourceOwner_e = 25;
pub const OWNER_SDCARD_CS: resourceOwner_e = 24;
pub const OWNER_SDCARD: resourceOwner_e = 23;
pub const OWNER_I2C_SDA: resourceOwner_e = 22;
pub const OWNER_I2C_SCL: resourceOwner_e = 21;
pub const OWNER_SPI_MOSI: resourceOwner_e = 20;
pub const OWNER_SPI_MISO: resourceOwner_e = 19;
pub const OWNER_SPI_SCK: resourceOwner_e = 18;
pub const OWNER_SYSTEM: resourceOwner_e = 17;
pub const OWNER_SONAR_ECHO: resourceOwner_e = 16;
pub const OWNER_SONAR_TRIGGER: resourceOwner_e = 15;
pub const OWNER_TIMER: resourceOwner_e = 14;
pub const OWNER_PINDEBUG: resourceOwner_e = 13;
pub const OWNER_SERIAL_RX: resourceOwner_e = 12;
pub const OWNER_SERIAL_TX: resourceOwner_e = 11;
pub const OWNER_ADC_RSSI: resourceOwner_e = 10;
pub const OWNER_ADC_EXT: resourceOwner_e = 9;
pub const OWNER_ADC_CURR: resourceOwner_e = 8;
pub const OWNER_ADC_BATT: resourceOwner_e = 7;
pub const OWNER_ADC: resourceOwner_e = 6;
pub const OWNER_LED: resourceOwner_e = 5;
pub const OWNER_SERVO: resourceOwner_e = 4;
pub const OWNER_MOTOR: resourceOwner_e = 3;
pub const OWNER_PPMINPUT: resourceOwner_e = 2;
pub const OWNER_PWMINPUT: resourceOwner_e = 1;
pub const OWNER_FREE: resourceOwner_e = 0;
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
// packet tag to specify IO pin
pub type IO_t = *mut libc::c_void;
// type specifying IO pin. Currently ioRec_t pointer, but this may change
// NONE initializer for ioTag_t variables
// NONE initializer for IO_t variable
// both ioTag_t and IO_t are guarantied to be zero if pinid is NONE (no pin)
// this simplifies initialization (globals are zeroed on start) and allows
//  omitting unused fields in structure initializers.
// it is also possible to use IO_t and ioTag_t as boolean value
//   TODO - this may conflict with requirement to generate warning/error on IO_t - ioTag_t assignment
//   IO_t being pointer is only possibility I know of ..
// pin config handling
// pin config is packed into ioConfig_t to decrease memory requirements
// IOCFG_x macros are defined for common combinations for all CPUs; this
//  helps masking CPU differences
pub type ioConfig_t = uint8_t;
pub type ioRec_t = ioRec_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ioRec_s {
    pub gpio: *mut GPIO_TypeDef,
    pub pin: uint16_t,
    pub owner: resourceOwner_e,
    pub index: uint8_t,
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
pub type rccPeriphTag_t = uint8_t;
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
// io ports defs are stored in array by index now
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ioPortDef_s {
    pub rcc: rccPeriphTag_t,
}
pub const RCC_AHB1: rcc_reg = 4;
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
pub type rcc_reg = libc::c_uint;
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
/* *
  * @}
  */
/* * @defgroup GPIO_LL_EF_Data_Access Data Access
  * @{
  */
/* *
  * @brief  Return full input data register value for a dedicated port.
  * @rmtoll IDR          IDy           LL_GPIO_ReadInputPort
  * @param  GPIOx GPIO Port
  * @retval Input data register value of port
  */
#[inline]
unsafe extern "C" fn LL_GPIO_ReadInputPort(mut GPIOx: *mut GPIO_TypeDef)
 -> uint32_t {
    return (*GPIOx).IDR;
}
/* *
  * @brief  Return full output data register value for a dedicated port.
  * @rmtoll ODR          ODy           LL_GPIO_ReadOutputPort
  * @param  GPIOx GPIO Port
  * @retval Output data register value of port
  */
#[inline]
unsafe extern "C" fn LL_GPIO_ReadOutputPort(mut GPIOx: *mut GPIO_TypeDef)
 -> uint32_t {
    return (*GPIOx).ODR;
}
/* *
  * @brief  Set several pins to high level on dedicated gpio port.
  * @rmtoll BSRR         BSy           LL_GPIO_SetOutputPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_GPIO_SetOutputPin(mut GPIOx: *mut GPIO_TypeDef,
                                          mut PinMask: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as *mut uint32_t, PinMask);
}
/* *
  * @brief  Set several pins to low level on dedicated gpio port.
  * @rmtoll BSRR         BRy           LL_GPIO_ResetOutputPin
  * @param  GPIOx GPIO Port
  * @param  PinMask This parameter can be a combination of the following values:
  *         @arg @ref LL_GPIO_PIN_0
  *         @arg @ref LL_GPIO_PIN_1
  *         @arg @ref LL_GPIO_PIN_2
  *         @arg @ref LL_GPIO_PIN_3
  *         @arg @ref LL_GPIO_PIN_4
  *         @arg @ref LL_GPIO_PIN_5
  *         @arg @ref LL_GPIO_PIN_6
  *         @arg @ref LL_GPIO_PIN_7
  *         @arg @ref LL_GPIO_PIN_8
  *         @arg @ref LL_GPIO_PIN_9
  *         @arg @ref LL_GPIO_PIN_10
  *         @arg @ref LL_GPIO_PIN_11
  *         @arg @ref LL_GPIO_PIN_12
  *         @arg @ref LL_GPIO_PIN_13
  *         @arg @ref LL_GPIO_PIN_14
  *         @arg @ref LL_GPIO_PIN_15
  *         @arg @ref LL_GPIO_PIN_ALL
  * @retval None
  */
#[inline]
unsafe extern "C" fn LL_GPIO_ResetOutputPin(mut GPIOx: *mut GPIO_TypeDef,
                                            mut PinMask: uint32_t) {
    ::core::ptr::write_volatile(&mut (*GPIOx).BSRR as *mut uint32_t,
                                PinMask << 16 as libc::c_int);
}
#[no_mangle]
pub static mut ioPortDefs: [ioPortDef_s; 6] =
    [{
         let mut init =
             ioPortDef_s{rcc:
                             (((RCC_AHB1 as libc::c_int) << 5 as libc::c_int)
                                  as libc::c_long |
                                  (16 as libc::c_int *
                                       (((0x1 as libc::c_uint) <<
                                             0 as libc::c_uint) as
                                            libc::c_long >
                                            65535 as libc::c_long) as
                                           libc::c_int) as libc::c_long +
                                      ((8 as libc::c_int *
                                            (((0x1 as libc::c_uint) <<
                                                  0 as libc::c_uint) as
                                                 libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (((0x1 as libc::c_uint)
                                                           <<
                                                           0 as libc::c_uint)
                                                          as libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >
                                                 255 as libc::c_int as
                                                     libc::c_long) as
                                                libc::c_int) as libc::c_long +
                                           (8 as libc::c_int as libc::c_long -
                                                90 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           0 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    0 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    0 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             0
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         4 as libc::c_int as
                                                             libc::c_long +
                                                         14 as libc::c_int as
                                                             libc::c_long |
                                                         1 as libc::c_int as
                                                             libc::c_long) -
                                                2 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           0 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    0 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    0 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             0
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         2 as libc::c_int as
                                                             libc::c_long +
                                                         1 as libc::c_int as
                                                             libc::c_long))))
                                 as rccPeriphTag_t,};
         init
     },
     {
         let mut init =
             ioPortDef_s{rcc:
                             (((RCC_AHB1 as libc::c_int) << 5 as libc::c_int)
                                  as libc::c_long |
                                  (16 as libc::c_int *
                                       (((0x1 as libc::c_uint) <<
                                             1 as libc::c_uint) as
                                            libc::c_long >
                                            65535 as libc::c_long) as
                                           libc::c_int) as libc::c_long +
                                      ((8 as libc::c_int *
                                            (((0x1 as libc::c_uint) <<
                                                  1 as libc::c_uint) as
                                                 libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (((0x1 as libc::c_uint)
                                                           <<
                                                           1 as libc::c_uint)
                                                          as libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >
                                                 255 as libc::c_int as
                                                     libc::c_long) as
                                                libc::c_int) as libc::c_long +
                                           (8 as libc::c_int as libc::c_long -
                                                90 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           1 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    1 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    1 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             1
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         4 as libc::c_int as
                                                             libc::c_long +
                                                         14 as libc::c_int as
                                                             libc::c_long |
                                                         1 as libc::c_int as
                                                             libc::c_long) -
                                                2 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           1 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    1 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    1 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             1
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         2 as libc::c_int as
                                                             libc::c_long +
                                                         1 as libc::c_int as
                                                             libc::c_long))))
                                 as rccPeriphTag_t,};
         init
     },
     {
         let mut init =
             ioPortDef_s{rcc:
                             (((RCC_AHB1 as libc::c_int) << 5 as libc::c_int)
                                  as libc::c_long |
                                  (16 as libc::c_int *
                                       (((0x1 as libc::c_uint) <<
                                             2 as libc::c_uint) as
                                            libc::c_long >
                                            65535 as libc::c_long) as
                                           libc::c_int) as libc::c_long +
                                      ((8 as libc::c_int *
                                            (((0x1 as libc::c_uint) <<
                                                  2 as libc::c_uint) as
                                                 libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (((0x1 as libc::c_uint)
                                                           <<
                                                           2 as libc::c_uint)
                                                          as libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >
                                                 255 as libc::c_int as
                                                     libc::c_long) as
                                                libc::c_int) as libc::c_long +
                                           (8 as libc::c_int as libc::c_long -
                                                90 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           2 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    2 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    2 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         4 as libc::c_int as
                                                             libc::c_long +
                                                         14 as libc::c_int as
                                                             libc::c_long |
                                                         1 as libc::c_int as
                                                             libc::c_long) -
                                                2 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           2 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    2 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    2 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         2 as libc::c_int as
                                                             libc::c_long +
                                                         1 as libc::c_int as
                                                             libc::c_long))))
                                 as rccPeriphTag_t,};
         init
     },
     {
         let mut init =
             ioPortDef_s{rcc:
                             (((RCC_AHB1 as libc::c_int) << 5 as libc::c_int)
                                  as libc::c_long |
                                  (16 as libc::c_int *
                                       (((0x1 as libc::c_uint) <<
                                             3 as libc::c_uint) as
                                            libc::c_long >
                                            65535 as libc::c_long) as
                                           libc::c_int) as libc::c_long +
                                      ((8 as libc::c_int *
                                            (((0x1 as libc::c_uint) <<
                                                  3 as libc::c_uint) as
                                                 libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint)
                                                          as libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >
                                                 255 as libc::c_int as
                                                     libc::c_long) as
                                                libc::c_int) as libc::c_long +
                                           (8 as libc::c_int as libc::c_long -
                                                90 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    3 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    3 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         4 as libc::c_int as
                                                             libc::c_long +
                                                         14 as libc::c_int as
                                                             libc::c_long |
                                                         1 as libc::c_int as
                                                             libc::c_long) -
                                                2 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           3 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    3 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    3 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         2 as libc::c_int as
                                                             libc::c_long +
                                                         1 as libc::c_int as
                                                             libc::c_long))))
                                 as rccPeriphTag_t,};
         init
     },
     {
         let mut init =
             ioPortDef_s{rcc:
                             (((RCC_AHB1 as libc::c_int) << 5 as libc::c_int)
                                  as libc::c_long |
                                  (16 as libc::c_int *
                                       (((0x1 as libc::c_uint) <<
                                             4 as libc::c_uint) as
                                            libc::c_long >
                                            65535 as libc::c_long) as
                                           libc::c_int) as libc::c_long +
                                      ((8 as libc::c_int *
                                            (((0x1 as libc::c_uint) <<
                                                  4 as libc::c_uint) as
                                                 libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (((0x1 as libc::c_uint)
                                                           <<
                                                           4 as libc::c_uint)
                                                          as libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >
                                                 255 as libc::c_int as
                                                     libc::c_long) as
                                                libc::c_int) as libc::c_long +
                                           (8 as libc::c_int as libc::c_long -
                                                90 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           4 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    4 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    4 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         4 as libc::c_int as
                                                             libc::c_long +
                                                         14 as libc::c_int as
                                                             libc::c_long |
                                                         1 as libc::c_int as
                                                             libc::c_long) -
                                                2 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           4 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    4 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    4 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             4
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         2 as libc::c_int as
                                                             libc::c_long +
                                                         1 as libc::c_int as
                                                             libc::c_long))))
                                 as rccPeriphTag_t,};
         init
     },
     {
         let mut init =
             ioPortDef_s{rcc:
                             (((RCC_AHB1 as libc::c_int) << 5 as libc::c_int)
                                  as libc::c_long |
                                  (16 as libc::c_int *
                                       (((0x1 as libc::c_uint) <<
                                             5 as libc::c_uint) as
                                            libc::c_long >
                                            65535 as libc::c_long) as
                                           libc::c_int) as libc::c_long +
                                      ((8 as libc::c_int *
                                            (((0x1 as libc::c_uint) <<
                                                  5 as libc::c_uint) as
                                                 libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (((0x1 as libc::c_uint)
                                                           <<
                                                           5 as libc::c_uint)
                                                          as libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >
                                                 255 as libc::c_int as
                                                     libc::c_long) as
                                                libc::c_int) as libc::c_long +
                                           (8 as libc::c_int as libc::c_long -
                                                90 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           5 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    5 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    5 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             5
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         4 as libc::c_int as
                                                             libc::c_long +
                                                         14 as libc::c_int as
                                                             libc::c_long |
                                                         1 as libc::c_int as
                                                             libc::c_long) -
                                                2 as libc::c_int as
                                                    libc::c_long /
                                                    ((((0x1 as libc::c_uint)
                                                           <<
                                                           5 as libc::c_uint)
                                                          as libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    5 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >>
                                                          8 as libc::c_int *
                                                              (((0x1 as
                                                                     libc::c_uint)
                                                                    <<
                                                                    5 as
                                                                        libc::c_uint)
                                                                   as
                                                                   libc::c_long
                                                                   *
                                                                   1 as
                                                                       libc::c_long
                                                                   >>
                                                                   16 as
                                                                       libc::c_int
                                                                       *
                                                                       (((0x1
                                                                              as
                                                                              libc::c_uint)
                                                                             <<
                                                                             5
                                                                                 as
                                                                                 libc::c_uint)
                                                                            as
                                                                            libc::c_long
                                                                            >
                                                                            65535
                                                                                as
                                                                                libc::c_long)
                                                                           as
                                                                           libc::c_int
                                                                   >
                                                                   255 as
                                                                       libc::c_int
                                                                       as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int)
                                                         /
                                                         2 as libc::c_int as
                                                             libc::c_long +
                                                         1 as libc::c_int as
                                                             libc::c_long))))
                                 as rccPeriphTag_t,};
         init
     }];
#[no_mangle]
pub unsafe extern "C" fn IO_Rec(mut io: IO_t) -> *mut ioRec_t {
    return io as *mut ioRec_t;
}
#[no_mangle]
pub unsafe extern "C" fn IO_GPIO(mut io: IO_t) -> *mut GPIO_TypeDef {
    let mut ioRec: *const ioRec_t = IO_Rec(io);
    return (*ioRec).gpio;
}
#[no_mangle]
pub unsafe extern "C" fn IO_Pin(mut io: IO_t) -> uint16_t {
    let mut ioRec: *const ioRec_t = IO_Rec(io);
    return (*ioRec).pin;
}
// port index, GPIOA == 0
#[no_mangle]
pub unsafe extern "C" fn IO_GPIOPortIdx(mut io: IO_t) -> libc::c_int {
    if io.is_null() { return -(1 as libc::c_int) }
    return ((IO_GPIO(io) as
                 size_t).wrapping_sub((0x40000000 as
                                           libc::c_uint).wrapping_add(0x20000
                                                                          as
                                                                          libc::c_uint).wrapping_add(0
                                                                                                         as
                                                                                                         libc::c_uint)
                                          as libc::c_ulong) >>
                10 as libc::c_int) as libc::c_int;
    // ports are 0x400 apart
}
#[no_mangle]
pub unsafe extern "C" fn IO_EXTI_PortSourceGPIO(mut io: IO_t) -> libc::c_int {
    return IO_GPIOPortIdx(io);
}
#[no_mangle]
pub unsafe extern "C" fn IO_GPIO_PortSource(mut io: IO_t) -> libc::c_int {
    return IO_GPIOPortIdx(io);
}
// zero based pin index
#[no_mangle]
pub unsafe extern "C" fn IO_GPIOPinIdx(mut io: IO_t) -> libc::c_int {
    if io.is_null() { return -(1 as libc::c_int) }
    return 31 as libc::c_int -
               (IO_Pin(io) as libc::c_uint).leading_zeros() as i32;
    // CLZ is a bit faster than FFS
}
#[no_mangle]
pub unsafe extern "C" fn IO_EXTI_PinSource(mut io: IO_t) -> libc::c_int {
    return IO_GPIOPinIdx(io);
}
#[no_mangle]
pub unsafe extern "C" fn IO_GPIO_PinSource(mut io: IO_t) -> libc::c_int {
    return IO_GPIOPinIdx(io);
}
// mask on stm32f103, bit index on stm32f303
#[no_mangle]
pub unsafe extern "C" fn IO_EXTI_Line(mut io: IO_t) -> uint32_t {
    if io.is_null() { return 0 as libc::c_int as uint32_t }
    return ((1 as libc::c_int) << IO_GPIOPinIdx(io)) as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn IORead(mut io: IO_t) -> bool {
    if io.is_null() { return 0 as libc::c_int != 0 }
    return LL_GPIO_ReadInputPort(IO_GPIO(io)) & IO_Pin(io) as libc::c_uint !=
               0;
}
#[no_mangle]
pub unsafe extern "C" fn IOWrite(mut io: IO_t, mut hi: bool) {
    if io.is_null() { return }
    LL_GPIO_SetOutputPin(IO_GPIO(io),
                         ((IO_Pin(io) as libc::c_int) <<
                              (if hi as libc::c_int != 0 {
                                   0 as libc::c_int
                               } else { 16 as libc::c_int })) as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn IOHi(mut io: IO_t) {
    if io.is_null() { return }
    LL_GPIO_SetOutputPin(IO_GPIO(io), IO_Pin(io) as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn IOLo(mut io: IO_t) {
    if io.is_null() { return }
    LL_GPIO_ResetOutputPin(IO_GPIO(io), IO_Pin(io) as uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn IOToggle(mut io: IO_t) {
    if io.is_null() { return }
    let mut mask: uint32_t = IO_Pin(io) as uint32_t;
    // Read pin state from ODR but write to BSRR because it only changes the pins
    // high in the mask value rather than all pins. XORing ODR directly risks
    // setting other pins incorrectly because it change all pins' state.
    if LL_GPIO_ReadOutputPort(IO_GPIO(io)) & mask != 0 {
        mask <<= 16 as libc::c_int
        // bit is set, shift mask to reset half
    }
    LL_GPIO_SetOutputPin(IO_GPIO(io), mask);
}
// claim IO pin, set owner and resources
#[no_mangle]
pub unsafe extern "C" fn IOInit(mut io: IO_t, mut owner: resourceOwner_e,
                                mut index: uint8_t) {
    if io.is_null() { return }
    let mut ioRec: *mut ioRec_t = IO_Rec(io);
    (*ioRec).owner = owner;
    (*ioRec).index = index;
}
#[no_mangle]
pub unsafe extern "C" fn IORelease(mut io: IO_t) {
    if io.is_null() { return }
    let mut ioRec: *mut ioRec_t = IO_Rec(io);
    (*ioRec).owner = OWNER_FREE;
}
#[no_mangle]
pub unsafe extern "C" fn IOGetOwner(mut io: IO_t) -> resourceOwner_e {
    if io.is_null() { return OWNER_FREE }
    let mut ioRec: *const ioRec_t = IO_Rec(io);
    return (*ioRec).owner;
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
// preprocessor is used to convert pinid to requested C data value
// compile-time error is generated if requested pin is not available (not set in TARGET_IO_PORTx)
// ioTag_t and IO_t is supported, but ioTag_t is preferred
// expand pinid to to ioTag_t
//speed is packed inside modebits 5 and 2,
// declare available IO pins. Available pins are specified per target
// unimplemented
#[no_mangle]
pub unsafe extern "C" fn IOIsFreeOrPreinit(mut io: IO_t) -> bool {
    let mut owner: resourceOwner_e = IOGetOwner(io);
    if owner as libc::c_uint == OWNER_FREE as libc::c_int as libc::c_uint ||
           owner as libc::c_uint ==
               OWNER_SPI_PREINIT_IPU as libc::c_int as libc::c_uint ||
           owner as libc::c_uint ==
               OWNER_SPI_PREINIT_OPU as libc::c_int as libc::c_uint {
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn IOConfigGPIO(mut io: IO_t, mut cfg: ioConfig_t) {
    IOConfigGPIOAF(io, cfg, 0 as libc::c_int as uint8_t);
}
#[no_mangle]
pub unsafe extern "C" fn IOConfigGPIOAF(mut io: IO_t, mut cfg: ioConfig_t,
                                        mut af: uint8_t) {
    if io.is_null() { return }
    let rcc: rccPeriphTag_t = ioPortDefs[IO_GPIOPortIdx(io) as usize].rcc;
    RCC_ClockCmd(rcc, ENABLE);
    let mut init: LL_GPIO_InitTypeDef =
        {
            let mut init =
                LL_GPIO_InitTypeDef{Pin: IO_Pin(io) as uint32_t,
                                    Mode:
                                        (cfg as libc::c_int >>
                                             0 as libc::c_int &
                                             0x3 as libc::c_int) as uint32_t,
                                    Speed:
                                        (cfg as libc::c_int >>
                                             2 as libc::c_int &
                                             0x3 as libc::c_int) as uint32_t,
                                    OutputType:
                                        (cfg as libc::c_int >>
                                             4 as libc::c_int &
                                             0x1 as libc::c_int) as uint32_t,
                                    Pull:
                                        (cfg as libc::c_int >>
                                             5 as libc::c_int &
                                             0x3 as libc::c_int) as uint32_t,
                                    Alternate: af as uint32_t,};
            init
        };
    LL_GPIO_Init(IO_GPIO(io), &mut init);
}
static mut ioDefUsedMask: [uint16_t; 4] =
    [0xffff as libc::c_int as uint16_t, 0xffff as libc::c_int as uint16_t,
     0xffff as libc::c_int as uint16_t,
     ((1 as libc::c_int) << 2 as libc::c_int) as uint16_t];
static mut ioDefUsedOffset: [uint8_t; 4] =
    [0 as libc::c_int as uint8_t,
     ((0xffff as libc::c_int -
           (0xffff as libc::c_int >> 1 as libc::c_int &
                0x77777777 as libc::c_int) -
           (0xffff as libc::c_int >> 2 as libc::c_int &
                0x33333333 as libc::c_int) -
           (0xffff as libc::c_int >> 3 as libc::c_int &
                0x11111111 as libc::c_int) +
           (0xffff as libc::c_int -
                (0xffff as libc::c_int >> 1 as libc::c_int &
                     0x77777777 as libc::c_int) -
                (0xffff as libc::c_int >> 2 as libc::c_int &
                     0x33333333 as libc::c_int) -
                (0xffff as libc::c_int >> 3 as libc::c_int &
                     0x11111111 as libc::c_int) >> 4 as libc::c_int) &
           0xf0f0f0f as libc::c_int) % 255 as libc::c_int) as uint8_t,
     ((0xffff as libc::c_int -
           (0xffff as libc::c_int >> 1 as libc::c_int &
                0x77777777 as libc::c_int) -
           (0xffff as libc::c_int >> 2 as libc::c_int &
                0x33333333 as libc::c_int) -
           (0xffff as libc::c_int >> 3 as libc::c_int &
                0x11111111 as libc::c_int) +
           (0xffff as libc::c_int -
                (0xffff as libc::c_int >> 1 as libc::c_int &
                     0x77777777 as libc::c_int) -
                (0xffff as libc::c_int >> 2 as libc::c_int &
                     0x33333333 as libc::c_int) -
                (0xffff as libc::c_int >> 3 as libc::c_int &
                     0x11111111 as libc::c_int) >> 4 as libc::c_int) &
           0xf0f0f0f as libc::c_int) % 255 as libc::c_int +
          (0xffff as libc::c_int -
               (0xffff as libc::c_int >> 1 as libc::c_int &
                    0x77777777 as libc::c_int) -
               (0xffff as libc::c_int >> 2 as libc::c_int &
                    0x33333333 as libc::c_int) -
               (0xffff as libc::c_int >> 3 as libc::c_int &
                    0x11111111 as libc::c_int) +
               (0xffff as libc::c_int -
                    (0xffff as libc::c_int >> 1 as libc::c_int &
                         0x77777777 as libc::c_int) -
                    (0xffff as libc::c_int >> 2 as libc::c_int &
                         0x33333333 as libc::c_int) -
                    (0xffff as libc::c_int >> 3 as libc::c_int &
                         0x11111111 as libc::c_int) >> 4 as libc::c_int) &
               0xf0f0f0f as libc::c_int) % 255 as libc::c_int) as uint8_t,
     ((0xffff as libc::c_int -
           (0xffff as libc::c_int >> 1 as libc::c_int &
                0x77777777 as libc::c_int) -
           (0xffff as libc::c_int >> 2 as libc::c_int &
                0x33333333 as libc::c_int) -
           (0xffff as libc::c_int >> 3 as libc::c_int &
                0x11111111 as libc::c_int) +
           (0xffff as libc::c_int -
                (0xffff as libc::c_int >> 1 as libc::c_int &
                     0x77777777 as libc::c_int) -
                (0xffff as libc::c_int >> 2 as libc::c_int &
                     0x33333333 as libc::c_int) -
                (0xffff as libc::c_int >> 3 as libc::c_int &
                     0x11111111 as libc::c_int) >> 4 as libc::c_int) &
           0xf0f0f0f as libc::c_int) % 255 as libc::c_int +
          (0xffff as libc::c_int -
               (0xffff as libc::c_int >> 1 as libc::c_int &
                    0x77777777 as libc::c_int) -
               (0xffff as libc::c_int >> 2 as libc::c_int &
                    0x33333333 as libc::c_int) -
               (0xffff as libc::c_int >> 3 as libc::c_int &
                    0x11111111 as libc::c_int) +
               (0xffff as libc::c_int -
                    (0xffff as libc::c_int >> 1 as libc::c_int &
                         0x77777777 as libc::c_int) -
                    (0xffff as libc::c_int >> 2 as libc::c_int &
                         0x33333333 as libc::c_int) -
                    (0xffff as libc::c_int >> 3 as libc::c_int &
                         0x11111111 as libc::c_int) >> 4 as libc::c_int) &
               0xf0f0f0f as libc::c_int) % 255 as libc::c_int +
          (0xffff as libc::c_int -
               (0xffff as libc::c_int >> 1 as libc::c_int &
                    0x77777777 as libc::c_int) -
               (0xffff as libc::c_int >> 2 as libc::c_int &
                    0x33333333 as libc::c_int) -
               (0xffff as libc::c_int >> 3 as libc::c_int &
                    0x11111111 as libc::c_int) +
               (0xffff as libc::c_int -
                    (0xffff as libc::c_int >> 1 as libc::c_int &
                         0x77777777 as libc::c_int) -
                    (0xffff as libc::c_int >> 2 as libc::c_int &
                         0x33333333 as libc::c_int) -
                    (0xffff as libc::c_int >> 3 as libc::c_int &
                         0x11111111 as libc::c_int) >> 4 as libc::c_int) &
               0xf0f0f0f as libc::c_int) % 255 as libc::c_int) as uint8_t];
#[no_mangle]
pub static mut ioRecs: [ioRec_t; 49] =
    [ioRec_t{gpio: 0 as *const GPIO_TypeDef as *mut GPIO_TypeDef,
             pin: 0,
             owner: OWNER_FREE,
             index: 0,}; 49];
// initialize all ioRec_t structures from ROM
// currently only bitmask is used, this may change in future
#[no_mangle]
pub unsafe extern "C" fn IOInitGlobal() {
    let mut ioRec: *mut ioRec_t =
        ioRecs.as_mut_ptr(); // ports are 0x400 apart
    let mut port: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while (port as libc::c_ulong) <
              (::core::mem::size_of::<[uint16_t; 4]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<uint16_t>()
                                                   as libc::c_ulong) {
        let mut pin: libc::c_uint = 0 as libc::c_int as libc::c_uint;
        while (pin as libc::c_ulong) <
                  (::core::mem::size_of::<uint16_t>() as
                       libc::c_ulong).wrapping_mul(8 as libc::c_int as
                                                       libc::c_ulong) {
            if ioDefUsedMask[port as usize] as libc::c_int &
                   (1 as libc::c_int) << pin != 0 {
                (*ioRec).gpio =
                    (0x40000000 as
                         libc::c_uint).wrapping_add(0x20000 as
                                                        libc::c_uint).wrapping_add(0
                                                                                       as
                                                                                       libc::c_uint).wrapping_add(port
                                                                                                                      <<
                                                                                                                      10
                                                                                                                          as
                                                                                                                          libc::c_int)
                        as *mut GPIO_TypeDef;
                (*ioRec).pin = ((1 as libc::c_int) << pin) as uint16_t;
                ioRec = ioRec.offset(1)
            }
            pin = pin.wrapping_add(1)
        }
        port = port.wrapping_add(1)
    };
}
#[no_mangle]
pub unsafe extern "C" fn IOGetByTag(mut tag: ioTag_t) -> IO_t {
    let portIdx: libc::c_int =
        (tag as libc::c_int >> 4 as libc::c_int) - 1 as libc::c_int;
    let pinIdx: libc::c_int = tag as libc::c_int & 0xf as libc::c_int;
    if portIdx < 0 as libc::c_int || portIdx >= 4 as libc::c_int {
        return 0 as *mut libc::c_void
    }
    // check if pin exists
    if ioDefUsedMask[portIdx as usize] as libc::c_int &
           (1 as libc::c_int) << pinIdx == 0 {
        return 0 as *mut libc::c_void
    }
    // count bits before this pin on single port
    let mut offset: libc::c_int =
        ((((1 as libc::c_int) << pinIdx) - 1 as libc::c_int &
              ioDefUsedMask[portIdx as usize] as libc::c_int) as
             libc::c_uint).count_ones() as i32;
    // and add port offset
    offset += ioDefUsedOffset[portIdx as usize] as libc::c_int;
    return ioRecs.as_mut_ptr().offset(offset as isize) as IO_t;
}
