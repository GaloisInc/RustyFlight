use ::libc;
extern "C" {
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
    // ST0 or ST4
    // ST2 or ST3
    // ST0 or ST1
    // ADC1_INxx channel number
    // index into DMA buffer in case of sparse channels
    #[no_mangle]
    fn adcDeviceByInstance(instance: *mut ADC_TypeDef) -> ADCDevice;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    stm32f722xx.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    30-December-2016
  * @brief   CMSIS Cortex-M7 Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral's registers declarations and bits definition
  *           - Macros to access peripheral�s registers hardware
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
/* * @addtogroup CMSIS_Device
  * @{
  */
/* * @addtogroup stm32f722xx
  * @{
  */
/* __cplusplus */
/* * @addtogroup Configuration_section_for_CMSIS
  * @{
  */
/* *
 * @brief STM32F7xx Interrupt Number Definition, according to the selected device
 *        in @ref Library_configuration_section
 */
/* *****  Cortex-M7 Processor Exceptions Numbers ****************************************************************/
/* !< 2 Non Maskable Interrupt                                          */
/* !< 4 Cortex-M7 Memory Management Interrupt                           */
/* !< 5 Cortex-M7 Bus Fault Interrupt                                   */
/* !< 6 Cortex-M7 Usage Fault Interrupt                                 */
/* !< 11 Cortex-M7 SV Call Interrupt                                    */
/* !< 12 Cortex-M7 Debug Monitor Interrupt                              */
/* !< 14 Cortex-M7 Pend SV Interrupt                                    */
/* !< 15 Cortex-M7 System Tick Interrupt                                */
/* *****  STM32 specific Interrupt Numbers **********************************************************************/
/* !< Window WatchDog Interrupt                                         */
/* !< PVD through EXTI Line detection Interrupt                         */
/* !< Tamper and TimeStamp interrupts through the EXTI line             */
/* !< RTC Wakeup interrupt through the EXTI line                        */
/* !< FLASH global Interrupt                                            */
/* !< RCC global Interrupt                                              */
/* !< EXTI Line0 Interrupt                                              */
/* !< EXTI Line1 Interrupt                                              */
/* !< EXTI Line2 Interrupt                                              */
/* !< EXTI Line3 Interrupt                                              */
/* !< EXTI Line4 Interrupt                                              */
/* !< DMA1 Stream 0 global Interrupt                                    */
/* !< DMA1 Stream 1 global Interrupt                                    */
/* !< DMA1 Stream 2 global Interrupt                                    */
/* !< DMA1 Stream 3 global Interrupt                                    */
/* !< DMA1 Stream 4 global Interrupt                                    */
/* !< DMA1 Stream 5 global Interrupt                                    */
/* !< DMA1 Stream 6 global Interrupt                                    */
/* !< ADC1, ADC2 and ADC3 global Interrupts                             */
/* !< CAN1 TX Interrupt                                                 */
/* !< CAN1 RX0 Interrupt                                                */
/* !< CAN1 RX1 Interrupt                                                */
/* !< CAN1 SCE Interrupt                                                */
/* !< External Line[9:5] Interrupts                                     */
/* !< TIM1 Break interrupt and TIM9 global interrupt                    */
/* !< TIM1 Update Interrupt and TIM10 global interrupt                  */
/* !< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
/* !< TIM1 Capture Compare Interrupt                                    */
/* !< TIM2 global Interrupt                                             */
/* !< TIM3 global Interrupt                                             */
/* !< TIM4 global Interrupt                                             */
/* !< I2C1 Event Interrupt                                              */
/* !< I2C1 Error Interrupt                                              */
/* !< I2C2 Event Interrupt                                              */
/* !< I2C2 Error Interrupt                                              */
/* !< SPI1 global Interrupt                                             */
/* !< SPI2 global Interrupt                                             */
/* !< USART1 global Interrupt                                           */
/* !< USART2 global Interrupt                                           */
/* !< USART3 global Interrupt                                           */
/* !< External Line[15:10] Interrupts                                   */
/* !< RTC Alarm (A and B) through EXTI Line Interrupt                   */
/* !< USB OTG FS Wakeup through EXTI line interrupt                     */
/* !< TIM8 Break Interrupt and TIM12 global interrupt                   */
/* !< TIM8 Update Interrupt and TIM13 global interrupt                  */
/* !< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
/* !< TIM8 Capture Compare Interrupt                                    */
/* !< DMA1 Stream7 Interrupt                                            */
/* !< FMC global Interrupt                                              */
/* !< SDMMC1 global Interrupt                                           */
/* !< TIM5 global Interrupt                                             */
/* !< SPI3 global Interrupt                                             */
/* !< UART4 global Interrupt                                            */
/* !< UART5 global Interrupt                                            */
/* !< TIM6 global and DAC1&2 underrun error  interrupts                 */
/* !< TIM7 global interrupt                                             */
/* !< DMA2 Stream 0 global Interrupt                                    */
/* !< DMA2 Stream 1 global Interrupt                                    */
/* !< DMA2 Stream 2 global Interrupt                                    */
/* !< DMA2 Stream 3 global Interrupt                                    */
/* !< DMA2 Stream 4 global Interrupt                                    */
/* !< Ethernet global Interrupt                                         */
/* !< Ethernet Wakeup through EXTI line Interrupt                       */
/* !< USB OTG FS global Interrupt                                       */
/* !< DMA2 Stream 5 global interrupt                                    */
/* !< DMA2 Stream 6 global interrupt                                    */
/* !< DMA2 Stream 7 global interrupt                                    */
/* !< USART6 global interrupt                                           */
/* !< I2C3 event interrupt                                              */
/* !< I2C3 error interrupt                                              */
/* !< USB OTG HS End Point 1 Out global interrupt                       */
/* !< USB OTG HS End Point 1 In global interrupt                        */
/* !< USB OTG HS Wakeup through EXTI interrupt                          */
/* !< USB OTG HS global interrupt                                       */
/* !< RNG global interrupt                                              */
/* !< FPU global interrupt                                              */
/* !< UART7 global interrupt                                            */
/* !< UART8 global interrupt                                            */
/* !< SPI4 global Interrupt                                             */
/* !< SPI5 global Interrupt                                             */
/* !< SAI1 global Interrupt                                             */
/* !< SAI2 global Interrupt                                             */
/* !< Quad SPI global interrupt                                         */
/* !< LP TIM1 interrupt                                                 */
/* !< SDMMC2 global Interrupt                                           */
/* *
  * @}
  */
/* *
 * @brief Configuration of the Cortex-M7 Processor and Core Peripherals
 */
/* !< Cortex-M7 revision r1p0                       */
/* !< CM7 provides an MPU                           */
/* !< CM7 uses 4 Bits for the Priority Levels       */
/* !< Set to 1 if different SysTick Config is used  */
/* !< FPU present                                   */
/* !< CM7 instruction cache present                 */
/* !< CM7 data cache present                        */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* *
  * @brief Analog to Digital Converter
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_TypeDef {
    pub SR: uint32_t,
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub JOFR1: uint32_t,
    pub JOFR2: uint32_t,
    pub JOFR3: uint32_t,
    pub JOFR4: uint32_t,
    pub HTR: uint32_t,
    pub LTR: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub JSQR: uint32_t,
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub DR: uint32_t,
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
pub type pgn_t = uint16_t;
pub type C2RustUnnamed = libc::c_uint;
pub const PGR_SIZE_SYSTEM_FLAG: C2RustUnnamed = 0;
pub const PGR_SIZE_MASK: C2RustUnnamed = 4095;
pub const PGR_PGN_VERSION_MASK: C2RustUnnamed = 61440;
pub const PGR_PGN_MASK: C2RustUnnamed = 4095;
// function that resets a single parameter group instance
pub type pgResetFunc
    =
    unsafe extern "C" fn(_: *mut libc::c_void, _: libc::c_int) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct pgRegistry_s {
    pub pgn: pgn_t,
    pub size: uint16_t,
    pub address: *mut uint8_t,
    pub copy: *mut uint8_t,
    pub ptr: *mut *mut uint8_t,
    pub reset: C2RustUnnamed_0,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_0 {
    pub ptr: *mut libc::c_void,
    pub fn_0: Option<pgResetFunc>,
}
pub type pgRegistry_t = pgRegistry_s;
pub type ioTag_t = uint8_t;
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 3;
pub const ADCDEV_3: ADCDevice = 2;
pub const ADCDEV_2: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcConfig_s {
    pub vbat: adcChannelConfig_t,
    pub rssi: adcChannelConfig_t,
    pub current: adcChannelConfig_t,
    pub external1: adcChannelConfig_t,
    pub device: int8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcChannelConfig_t {
    pub enabled: bool,
    pub ioTag: ioTag_t,
}
pub type adcConfig_t = adcConfig_s;
/* base */
/* size */
// The parameter group number, the top 4 bits are reserved for version
// Size of the group in RAM, the top 4 bits are reserved for flags
// Address of the group in RAM.
// Address of the copy in RAM.
// The pointer to update after loading the record into ram.
// Pointer to init template
// Pointer to pgResetFunc
// ADCDevice
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
pub static mut adcConfig_System: adcConfig_t =
    adcConfig_t{vbat: adcChannelConfig_t{enabled: false, ioTag: 0,},
                rssi: adcChannelConfig_t{enabled: false, ioTag: 0,},
                current: adcChannelConfig_t{enabled: false, ioTag: 0,},
                external1: adcChannelConfig_t{enabled: false, ioTag: 0,},
                device: 0,};
#[no_mangle]
pub static mut adcConfig_Copy: adcConfig_t =
    adcConfig_t{vbat: adcChannelConfig_t{enabled: false, ioTag: 0,},
                rssi: adcChannelConfig_t{enabled: false, ioTag: 0,},
                current: adcChannelConfig_t{enabled: false, ioTag: 0,},
                external1: adcChannelConfig_t{enabled: false, ioTag: 0,},
                device: 0,};
#[no_mangle]
#[link_section = ".pg_registry"]
#[used]
pub static mut adcConfig_Registry: pgRegistry_t =
    unsafe {
        {
            let mut init =
                pgRegistry_s{pgn:
                                 (510 as libc::c_int |
                                      (0 as libc::c_int) << 12 as libc::c_int)
                                     as pgn_t,
                             size:
                                 (::core::mem::size_of::<adcConfig_t>() as
                                      libc::c_ulong |
                                      PGR_SIZE_SYSTEM_FLAG as libc::c_int as
                                          libc::c_ulong) as uint16_t,
                             address:
                                 &adcConfig_System as *const adcConfig_t as
                                     *mut adcConfig_t as *mut uint8_t,
                             copy:
                                 &adcConfig_Copy as *const adcConfig_t as
                                     *mut adcConfig_t as *mut uint8_t,
                             ptr:
                                 0 as *const *mut uint8_t as
                                     *mut *mut uint8_t,
                             reset:
                                 C2RustUnnamed_0{fn_0:
                                                     ::core::mem::transmute::<Option<unsafe extern "C" fn(_:
                                                                                                              *mut adcConfig_t)
                                                                                         ->
                                                                                             ()>,
                                                                              Option<pgResetFunc>>(Some(pgResetFn_adcConfig
                                                                                                            as
                                                                                                            unsafe extern "C" fn(_:
                                                                                                                                     *mut adcConfig_t)
                                                                                                                ->
                                                                                                                    ())),},};
            init
        }
    };
#[no_mangle]
pub unsafe extern "C" fn pgResetFn_adcConfig(mut adcConfig:
                                                 *mut adcConfig_t) {
    (*adcConfig).device =
        (adcDeviceByInstance((0x40000000 as
                                  libc::c_uint).wrapping_add(0x10000 as
                                                                 libc::c_uint).wrapping_add(0x2000
                                                                                                as
                                                                                                libc::c_uint)
                                 as *mut ADC_TypeDef) as libc::c_int +
             1 as libc::c_int) as int8_t;
}
// USE_ADC
