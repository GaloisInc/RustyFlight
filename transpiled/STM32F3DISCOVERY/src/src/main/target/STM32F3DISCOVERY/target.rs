use ::libc;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* *
  ******************************************************************************
  * @file    stm32f30x.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    28-March-2014
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer Header File. 
  *          This file contains all the peripheral registers definitions, bits 
  *          definitions and memory mapping for STM32F30x devices.
  *            
  *          The file is the unique include file that the application programmer
  *          is using in the C source code, usually in main.c. This file contains:
  *           - Configuration section that allows to select:
  *              - The device used in the target application
  *              - To use or not the peripheral's drivers in application code(i.e.
  *                code will be based on direct access to peripheral's registers
  *                rather than drivers API), this option is controlled by 
  *                "#define USE_STDPERIPH_DRIVER"
  *              - To change few application-specific parameters such as the HSE 
  *                crystal frequency
  *           - Data structures and the address mapping for all peripherals
  *           - Peripheral registers declarations and bits definition
  *           - Macros to access peripheral registers hardware
  *  
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
/* * @addtogroup CMSIS
  * @{
  */
/* * @addtogroup stm32f30x
  * @{
  */
/* __cplusplus */
/* * @addtogroup Library_configuration_section
  * @{
  */
/* Uncomment the line below according to the target STM32 device used in your
   application 
  */
/*  Tip: To avoid modifying this file each time you need to switch between these
        devices, you can define the device in your toolchain compiler preprocessor.
  */
/* Old STM32F30X definition, maintained for legacy purpose */
/* STM32F30X */
/* USE_STDPERIPH_DRIVER */
/* *
 * @brief In the following line adjust the value of External High Speed oscillator (HSE)
   used in your application 
   
   Tip: To avoid modifying this file each time you need to use different HSE, you
        can define the HSE value in your toolchain compiler preprocessor.
  */
/* HSE_VALUE */
/* *
 * @brief In the following line adjust the External High Speed oscillator (HSE) Startup 
   Timeout value 
   */
/* !< Time out for HSE start up */
/* HSE_STARTUP_TIMEOUT */
/* *
 * @brief In the following line adjust the Internal High Speed oscillator (HSI) Startup 
   Timeout value 
   */
/* !< Time out for HSI start up */
/* HSI_STARTUP_TIMEOUT */
/* HSI_VALUE */                      /*!< Value of the Internal High Speed oscillator in Hz.
                                            The real value may vary depending on the variations
                                             in voltage and temperature.  */
/* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
/* !< Value of the External Low Speed oscillator in Hz */
/* LSE_VALUE */
/* *
 * @brief STM32F30x Standard Peripherals Library version number V1.1.0
   */
/* !< [31:24] main version */
/* !< [23:16] sub1 version */
/* !< [15:8]  sub2 version */
/* !< [7:0]  release candidate */
/* *
  * @}
  */
/* * @addtogroup Configuration_section_for_CMSIS
  * @{
  */
/* *
 * @brief Configuration of the Cortex-M4 Processor and Core Peripherals 
 */
/* !< Core revision r0p1                            */
/* !< STM32F30X provide an MPU */
/* !< STM32F30X uses 4 Bits for the Priority Levels */
/* !< Set to 1 if different SysTick Config is used */
/* !< STM32F30X provide an FPU */
/* *
 * @brief STM32F30X Interrupt Number Definition, according to the selected device 
 *        in @ref Library_configuration_section 
 */
/* *****  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
/* !< 2 Non Maskable Interrupt                                          */
/* !< 4 Cortex-M4 Memory Management Interrupt                           */
/* !< 5 Cortex-M4 Bus Fault Interrupt                                   */
/* !< 6 Cortex-M4 Usage Fault Interrupt                                 */
/* !< 11 Cortex-M4 SV Call Interrupt                                    */
/* !< 12 Cortex-M4 Debug Monitor Interrupt                              */
/* !< 14 Cortex-M4 Pend SV Interrupt                                    */
/* !< 15 Cortex-M4 System Tick Interrupt                                */
/* *****  STM32 specific Interrupt Numbers **********************************************************************/
/* !< Window WatchDog Interrupt                                         */
/* !< PVD through EXTI Line detection Interrupt                         */
/* !< Tamper and TimeStamp interrupts                                   */
/* !< RTC Wakeup interrupt through the EXTI lines 17, 19 & 20           */
/* !< FLASH global Interrupt                                            */
/* !< RCC global Interrupt                                              */
/* !< EXTI Line0 Interrupt                                              */
/* !< EXTI Line1 Interrupt                                              */
/* !< EXTI Line2 Interrupt and Touch Sense Interrupt                    */
/* !< EXTI Line3 Interrupt                                              */
/* !< EXTI Line4 Interrupt                                              */
/* !< DMA1 Channel 1 Interrupt                                          */
/* !< DMA1 Channel 2 Interrupt                                          */
/* !< DMA1 Channel 3 Interrupt                                          */
/* !< DMA1 Channel 4 Interrupt                                          */
/* !< DMA1 Channel 5 Interrupt                                          */
/* !< DMA1 Channel 6 Interrupt                                          */
/* !< DMA1 Channel 7 Interrupt                                          */
/* !< ADC1 & ADC2 Interrupts                                            */
/* !< USB Device High Priority or CAN1 TX Interrupts                    */
/* !< USB Device Low Priority or CAN1 RX0 Interrupts                    */
/* !< CAN1 RX1 Interrupt                                                */
/* !< CAN1 SCE Interrupt                                                */
/* !< External Line[9:5] Interrupts                                     */
/* !< TIM1 Break and TIM15 Interrupts                                   */
/* !< TIM1 Update and TIM16 Interrupts                                  */
/* !< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
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
/* !< USB Wakeup Interrupt                                              */
/* !< TIM8 Break Interrupt                                              */
/* !< TIM8 Update Interrupt                                             */
/* !< TIM8 Trigger and Commutation Interrupt                            */
/* !< TIM8 Capture Compare Interrupt                                    */
/* !< ADC3 global Interrupt                                             */
/* !< SPI3 global Interrupt                                             */
/* !< UART4 global Interrupt                                            */
/* !< UART5 global Interrupt                                            */
/* !< TIM6 global and DAC1&2 underrun error  interrupts                 */
/* !< TIM7 global Interrupt                                             */
/* !< DMA2 Channel 1 global Interrupt                                   */
/* !< DMA2 Channel 2 global Interrupt                                   */
/* !< DMA2 Channel 3 global Interrupt                                   */
/* !< DMA2 Channel 4 global Interrupt                                   */
/* !< DMA2 Channel 5 global Interrupt                                   */
/* !< ADC4  global Interrupt                                            */
/* !< COMP1, COMP2 and COMP3 global Interrupt                           */
/* !< COMP5, COMP6 and COMP4 global Interrupt                           */
/* !< COMP7 global Interrupt                                            */
/* !< USB High Priority global Interrupt remap                          */
/* !< USB Low Priority global Interrupt  remap                          */
/* !< USB Wakeup Interrupt remap                                        */
/* !< Floating point Interrupt                                          */
/* STM32F303xC */
/* STM32F334x8 */
/* STM32F302x8 */
/* *
  * @}
  */
/* * @addtogroup Exported_types
  * @{
  */
/* !< STM32F10x Standard Peripheral Library old types (maintained for legacy purpose) */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* !< Read Only */
/* *
  * @}
  */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* * 
  * @brief Analog to Digital Converter  
  */
/* !< ADC Interrupt and Status Register,                 Address offset: 0x00 */
/* !< ADC Interrupt Enable Register,                     Address offset: 0x04 */
/* !< ADC control register,                              Address offset: 0x08 */
/* !< ADC Configuration register,                        Address offset: 0x0C */
/* !< Reserved, 0x010                                                         */
/* !< ADC sample time register 1,                        Address offset: 0x14 */
/* !< ADC sample time register 2,                        Address offset: 0x18 */
/* !< Reserved, 0x01C                                                         */
/* !< ADC watchdog threshold register 1,                 Address offset: 0x20 */
/* !< ADC watchdog threshold register 2,                 Address offset: 0x24 */
/* !< ADC watchdog threshold register 3,                 Address offset: 0x28 */
/* !< Reserved, 0x02C                                                         */
/* !< ADC regular sequence register 1,                   Address offset: 0x30 */
/* !< ADC regular sequence register 2,                   Address offset: 0x34 */
/* !< ADC regular sequence register 3,                   Address offset: 0x38 */
/* !< ADC regular sequence register 4,                   Address offset: 0x3C */
/* !< ADC regular data register,                         Address offset: 0x40 */
/* !< Reserved, 0x044                                                         */
/* !< Reserved, 0x048                                                         */
/* !< ADC injected sequence register,                    Address offset: 0x4C */
/* !< Reserved, 0x050 - 0x05C                                                 */
/* !< ADC offset register 1,                             Address offset: 0x60 */
/* !< ADC offset register 2,                             Address offset: 0x64 */
/* !< ADC offset register 3,                             Address offset: 0x68 */
/* !< ADC offset register 4,                             Address offset: 0x6C */
/* !< Reserved, 0x070 - 0x07C                                                 */
/* !< ADC injected data register 1,                      Address offset: 0x80 */
/* !< ADC injected data register 2,                      Address offset: 0x84 */
/* !< ADC injected data register 3,                      Address offset: 0x88 */
/* !< ADC injected data register 4,                      Address offset: 0x8C */
/* !< Reserved, 0x090 - 0x09C                                                 */
/* !< ADC  Analog Watchdog 2 Configuration Register,     Address offset: 0xA0 */
/* !< ADC  Analog Watchdog 3 Configuration Register,     Address offset: 0xA4 */
/* !< Reserved, 0x0A8                                                         */
/* !< Reserved, 0x0AC                                                         */
/* !< ADC  Differential Mode Selection Register,         Address offset: 0xB0 */
/* !< ADC  Calibration Factors,                          Address offset: 0xB4 */
/* !< ADC Common status register,                  Address offset: ADC1/3 base address + 0x300 */
/* !< Reserved, ADC1/3 base address + 0x304                                                    */
/* !< ADC common control register,                 Address offset: ADC1/3 base address + 0x308 */
/* !< ADC common regular data register for dual
                                     modes,                                       Address offset: ADC1/3 base address + 0x30A */
/* * 
  * @brief Controller Area Network TxMailBox 
  */
/* !< CAN TX mailbox identifier register */
/* !< CAN mailbox data length control and time stamp register */
/* !< CAN mailbox data low register */
/* !< CAN mailbox data high register */
/* * 
  * @brief Controller Area Network FIFOMailBox 
  */
/* !< CAN receive FIFO mailbox identifier register */
/* !< CAN receive FIFO mailbox data length control and time stamp register */
/* !< CAN receive FIFO mailbox data low register */
/* !< CAN receive FIFO mailbox data high register */
/* * 
  * @brief Controller Area Network FilterRegister 
  */
/* !< CAN Filter bank register 1 */
/* !< CAN Filter bank register 1 */
/* * 
  * @brief Controller Area Network 
  */
/* !< CAN master control register,         Address offset: 0x00          */
/* !< CAN master status register,          Address offset: 0x04          */
/* !< CAN transmit status register,        Address offset: 0x08          */
/* !< CAN receive FIFO 0 register,         Address offset: 0x0C          */
/* !< CAN receive FIFO 1 register,         Address offset: 0x10          */
/* !< CAN interrupt enable register,       Address offset: 0x14          */
/* !< CAN error status register,           Address offset: 0x18          */
/* !< CAN bit timing register,             Address offset: 0x1C          */
/* !< Reserved, 0x020 - 0x17F                                            */
/* !< CAN Tx MailBox,                      Address offset: 0x180 - 0x1AC */
/* !< CAN FIFO MailBox,                    Address offset: 0x1B0 - 0x1CC */
/* !< Reserved, 0x1D0 - 0x1FF                                            */
/* !< CAN filter master register,          Address offset: 0x200         */
/* !< CAN filter mode register,            Address offset: 0x204         */
/* !< Reserved, 0x208                                                    */
/* !< CAN filter scale register,           Address offset: 0x20C         */
/* !< Reserved, 0x210                                                    */
/* !< CAN filter FIFO assignment register, Address offset: 0x214         */
/* !< Reserved, 0x218                                                    */
/* !< CAN filter activation register,      Address offset: 0x21C         */
/* !< Reserved, 0x220-0x23F                                              */
/* !< CAN Filter Register,                 Address offset: 0x240-0x31C   */
/* * 
  * @brief Analog Comparators 
  */
/* !< Comparator control Status register, Address offset: 0x00 */
/* * 
  * @brief CRC calculation unit 
  */
/* !< CRC Data register,                           Address offset: 0x00 */
/* !< CRC Independent data register,               Address offset: 0x04 */
/* !< Reserved,                                                    0x05 */
/* !< Reserved,                                                    0x06 */
/* !< CRC Control register,                        Address offset: 0x08 */
/* !< Reserved,                                                    0x0C */
/* !< Initial CRC value register,                  Address offset: 0x10 */
/* !< CRC polynomial register,                     Address offset: 0x14 */
/* * 
  * @brief Digital to Analog Converter
  */
/* !< DAC control register,                                    Address offset: 0x00 */
/* !< DAC software trigger register,                           Address offset: 0x04 */
/* !< DAC channel1 12-bit right-aligned data holding register, Address offset: 0x08 */
/* !< DAC channel1 12-bit left aligned data holding register,  Address offset: 0x0C */
/* !< DAC channel1 8-bit right aligned data holding register,  Address offset: 0x10 */
/* !< DAC channel2 12-bit right aligned data holding register, Address offset: 0x14 */
/* !< DAC channel2 12-bit left aligned data holding register,  Address offset: 0x18 */
/* !< DAC channel2 8-bit right-aligned data holding register,  Address offset: 0x1C */
/* !< Dual DAC 12-bit right-aligned data holding register,     Address offset: 0x20 */
/* !< DUAL DAC 12-bit left aligned data holding register,      Address offset: 0x24 */
/* !< DUAL DAC 8-bit right aligned data holding register,      Address offset: 0x28 */
/* !< DAC channel1 data output register,                       Address offset: 0x2C */
/* !< DAC channel2 data output register,                       Address offset: 0x30 */
/* !< DAC status register,                                     Address offset: 0x34 */
/* * 
  * @brief Debug MCU
  */
/* !< MCU device ID code,               Address offset: 0x00 */
/* !< Debug MCU configuration register, Address offset: 0x04 */
/* !< Debug MCU APB1 freeze register,   Address offset: 0x08 */
/* !< Debug MCU APB2 freeze register,   Address offset: 0x0C */
/* * 
  * @brief DMA Controller
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint32_t,
    pub SMCR: uint32_t,
    pub DIER: uint32_t,
    pub SR: uint32_t,
    pub EGR: uint32_t,
    pub CCMR1: uint32_t,
    pub CCMR2: uint32_t,
    pub CCER: uint32_t,
    pub CNT: uint32_t,
    pub PSC: uint16_t,
    pub RESERVED9: uint16_t,
    pub ARR: uint32_t,
    pub RCR: uint16_t,
    pub RESERVED10: uint16_t,
    pub CCR1: uint32_t,
    pub CCR2: uint32_t,
    pub CCR3: uint32_t,
    pub CCR4: uint32_t,
    pub BDTR: uint32_t,
    pub DCR: uint16_t,
    pub RESERVED12: uint16_t,
    pub DMAR: uint16_t,
    pub RESERVED13: uint16_t,
    pub OR: uint16_t,
    pub CCMR3: uint32_t,
    pub CCR5: uint32_t,
    pub CCR6: uint32_t,
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
// IO pin identification
// make sure that ioTag_t can't be assigned into IO_t without warning
pub type ioTag_t = uint8_t;
pub type timerUsageFlag_e = libc::c_uint;
pub const TIM_USE_BEEPER: timerUsageFlag_e = 64;
pub const TIM_USE_TRANSPONDER: timerUsageFlag_e = 32;
pub const TIM_USE_LED: timerUsageFlag_e = 16;
pub const TIM_USE_SERVO: timerUsageFlag_e = 8;
pub const TIM_USE_MOTOR: timerUsageFlag_e = 4;
pub const TIM_USE_PWM: timerUsageFlag_e = 2;
pub const TIM_USE_PPM: timerUsageFlag_e = 1;
pub const TIM_USE_NONE: timerUsageFlag_e = 0;
pub const TIM_USE_ANY: timerUsageFlag_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerHardware_s {
    pub tim: *mut TIM_TypeDef,
    pub tag: ioTag_t,
    pub channel: uint8_t,
    pub usageFlags: timerUsageFlag_e,
    pub output: uint8_t,
    pub alternateFunction: uint8_t,
    pub dmaRef: *mut DMA_Channel_TypeDef,
    pub dmaIrqHandler: uint8_t,
    pub dmaTimUPRef: *mut DMA_Channel_TypeDef,
    pub dmaTimUPIrqHandler: uint8_t,
}
pub type timerHardware_t = timerHardware_s;
pub type C2RustUnnamed = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed = 0;
pub const DMA1_CH2_HANDLER: C2RustUnnamed_0 = 2;
pub const DMA1_CH1_HANDLER: C2RustUnnamed_0 = 1;
pub const DMA1_CH7_HANDLER: C2RustUnnamed_0 = 7;
pub const DMA1_CH5_HANDLER: C2RustUnnamed_0 = 5;
pub const DMA1_CH4_HANDLER: C2RustUnnamed_0 = 4;
pub const DMA1_CH3_HANDLER: C2RustUnnamed_0 = 3;
pub const DMA2_CH1_HANDLER: C2RustUnnamed_0 = 8;
pub const DMA2_CH5_HANDLER: C2RustUnnamed_0 = 12;
pub const DMA2_CH3_HANDLER: C2RustUnnamed_0 = 10;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const DMA_LAST_HANDLER: C2RustUnnamed_0 = 12;
pub const DMA2_CH4_HANDLER: C2RustUnnamed_0 = 11;
pub const DMA2_CH2_HANDLER: C2RustUnnamed_0 = 9;
pub const DMA1_CH6_HANDLER: C2RustUnnamed_0 = 6;
pub const DMA_NONE: C2RustUnnamed_0 = 0;
// TIMUP
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
// Initialized in run_static_initializers
#[no_mangle]
pub static mut timerHardware: [timerHardware_t; 14] =
    [timerHardware_t{tim: 0 as *mut TIM_TypeDef,
                     tag: 0,
                     channel: 0,
                     usageFlags: TIM_USE_ANY,
                     output: 0,
                     alternateFunction: 0,
                     dmaRef: 0 as *mut DMA_Channel_TypeDef,
                     dmaIrqHandler: 0,
                     dmaTimUPRef: 0 as *mut DMA_Channel_TypeDef,
                     dmaTimUPIrqHandler: 0,}; 14];
unsafe extern "C" fn run_static_initializers() {
    timerHardware =
        [{
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x4400
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 8 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags:
                                     (TIM_USE_PPM as libc::c_int |
                                          TIM_USE_LED as libc::c_int) as
                                         timerUsageFlag_e,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x4800
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 9 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x8
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x8
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x2c00
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 8 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x6 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x3400
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 6 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x4 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x430
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA2_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x408
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA2_CH1_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x3400
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 7 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x4 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x458
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA2_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x408
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA2_CH1_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x10000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x3400
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((2 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 8 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_MOTOR,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x4 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x408
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA2_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x408
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA2_CH1_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x400 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 1 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0xc as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x400 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 4 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef: 0 as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler: 0 as libc::c_int as uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x30
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH3_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((3 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          12 as libc::c_int) as ioTag_t,
                                 channel:
                                     0 as libc::c_int as uint16_t as uint8_t,
                                 usageFlags: TIM_USE_BEEPER,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x8
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((3 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          13 as libc::c_int) as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x44
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH4_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((3 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          14 as libc::c_int) as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x58
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH5_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x800 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((3 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          15 as libc::c_int) as ioTag_t,
                                 channel:
                                     0xc as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x2 as libc::c_int as uint8_t,
                                 dmaRef: 0 as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler: 0 as libc::c_int as uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 1 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x4 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x80
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH7_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         },
         {
             let mut init =
                 timerHardware_s{tim:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint)
                                         as *mut TIM_TypeDef,
                                 tag:
                                     ((0 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int | 2 as libc::c_int)
                                         as ioTag_t,
                                 channel:
                                     0x8 as libc::c_int as uint16_t as
                                         uint8_t,
                                 usageFlags: TIM_USE_ANY,
                                 output:
                                     (TIMER_OUTPUT_NONE as libc::c_int |
                                          0 as libc::c_int) as uint8_t,
                                 alternateFunction:
                                     0x1 as libc::c_int as uint8_t,
                                 dmaRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x8
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaIrqHandler:
                                     DMA1_CH1_HANDLER as libc::c_int as
                                         uint8_t,
                                 dmaTimUPRef:
                                     (0x40000000 as libc::c_int as
                                          uint32_t).wrapping_add(0x20000 as
                                                                     libc::c_int
                                                                     as
                                                                     libc::c_uint).wrapping_add(0x1c
                                                                                                    as
                                                                                                    libc::c_int
                                                                                                    as
                                                                                                    libc::c_uint)
                                         as *mut DMA_Channel_TypeDef,
                                 dmaTimUPIrqHandler:
                                     DMA1_CH2_HANDLER as libc::c_int as
                                         uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
