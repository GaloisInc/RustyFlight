use ::libc;
extern "C" {
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    /* *
  * @}
  */
    /* * @defgroup USART_Global_definition 
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
    /* Initialization and Configuration functions *********************************/
    /* STOP Mode functions ********************************************************/
    /* AutoBaudRate functions *****************************************************/
    /* Data transfers functions ***************************************************/
    /* Multi-Processor Communication functions ************************************/
    /* LIN mode functions *********************************************************/
    /* Half-duplex mode function **************************************************/
    /* Smartcard mode functions ***************************************************/
    /* IrDA mode functions ********************************************************/
    /* RS485 mode functions *******************************************************/
    /* DMA transfers management functions *****************************************/
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn USART_ClearITPendingBit(USARTx: *mut USART_TypeDef,
                               USART_IT: uint32_t);
    #[no_mangle]
    fn USART_ITConfig(USARTx: *mut USART_TypeDef, USART_IT: uint32_t,
                      NewState: FunctionalState);
    #[no_mangle]
    fn USART_SendData(USARTx: *mut USART_TypeDef, Data: uint16_t);
    #[no_mangle]
    fn DMA_Cmd(DMAy_Channelx: *mut DMA_Channel_TypeDef,
               NewState: FunctionalState);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn dmaGetIdentifier(channel: *const DMA_Channel_TypeDef)
     -> dmaIdentifier_e;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    static mut uartDevmap: [*mut uartDevice_t; 0];
    #[no_mangle]
    static uartVTable: [serialPortVTable; 0];
    #[no_mangle]
    fn uartTryStartTxDMA(s: *mut uartPort_t);
}
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
pub type IRQn = libc::c_int;
/* !< Floating point Interrupt                                          */
/* STM32F303xC */
/* STM32F334x8 */
/* STM32F302x8 */
/* !< USB Wakeup Interrupt remap                                        */
pub const FPU_IRQn: IRQn = 81;
/* !< USB Low Priority global Interrupt  remap                          */
pub const USBWakeUp_RMP_IRQn: IRQn = 76;
/* !< USB High Priority global Interrupt remap                          */
pub const USB_LP_IRQn: IRQn = 75;
/* !< COMP7 global Interrupt                                            */
pub const USB_HP_IRQn: IRQn = 74;
/* !< COMP5, COMP6 and COMP4 global Interrupt                           */
pub const COMP7_IRQn: IRQn = 66;
/* !< COMP1, COMP2 and COMP3 global Interrupt                           */
pub const COMP4_5_6_IRQn: IRQn = 65;
/* !< ADC4  global Interrupt                                            */
pub const COMP1_2_3_IRQn: IRQn = 64;
/* !< DMA2 Channel 5 global Interrupt                                   */
pub const ADC4_IRQn: IRQn = 61;
/* !< DMA2 Channel 4 global Interrupt                                   */
pub const DMA2_Channel5_IRQn: IRQn = 60;
/* !< DMA2 Channel 3 global Interrupt                                   */
pub const DMA2_Channel4_IRQn: IRQn = 59;
/* !< DMA2 Channel 2 global Interrupt                                   */
pub const DMA2_Channel3_IRQn: IRQn = 58;
/* !< DMA2 Channel 1 global Interrupt                                   */
pub const DMA2_Channel2_IRQn: IRQn = 57;
/* !< TIM7 global Interrupt                                             */
pub const DMA2_Channel1_IRQn: IRQn = 56;
/* !< TIM6 global and DAC1&2 underrun error  interrupts                 */
pub const TIM7_IRQn: IRQn = 55;
/* !< UART5 global Interrupt                                            */
pub const TIM6_DAC_IRQn: IRQn = 54;
/* !< UART4 global Interrupt                                            */
pub const UART5_IRQn: IRQn = 53;
/* !< SPI3 global Interrupt                                             */
pub const UART4_IRQn: IRQn = 52;
/* !< ADC3 global Interrupt                                             */
pub const SPI3_IRQn: IRQn = 51;
/* !< TIM8 Capture Compare Interrupt                                    */
pub const ADC3_IRQn: IRQn = 47;
/* !< TIM8 Trigger and Commutation Interrupt                            */
pub const TIM8_CC_IRQn: IRQn = 46;
/* !< TIM8 Update Interrupt                                             */
pub const TIM8_TRG_COM_IRQn: IRQn = 45;
/* !< TIM8 Break Interrupt                                              */
pub const TIM8_UP_IRQn: IRQn = 44;
/* !< USB Wakeup Interrupt                                              */
pub const TIM8_BRK_IRQn: IRQn = 43;
/* !< RTC Alarm (A and B) through EXTI Line Interrupt                   */
pub const USBWakeUp_IRQn: IRQn = 42;
/* !< External Line[15:10] Interrupts                                   */
pub const RTC_Alarm_IRQn: IRQn = 41;
/* !< USART3 global Interrupt                                           */
pub const EXTI15_10_IRQn: IRQn = 40;
/* !< USART2 global Interrupt                                           */
pub const USART3_IRQn: IRQn = 39;
/* !< USART1 global Interrupt                                           */
pub const USART2_IRQn: IRQn = 38;
/* !< SPI2 global Interrupt                                             */
pub const USART1_IRQn: IRQn = 37;
/* !< SPI1 global Interrupt                                             */
pub const SPI2_IRQn: IRQn = 36;
/* !< I2C2 Error Interrupt                                              */
pub const SPI1_IRQn: IRQn = 35;
/* !< I2C2 Event Interrupt                                              */
pub const I2C2_ER_IRQn: IRQn = 34;
/* !< I2C1 Error Interrupt                                              */
pub const I2C2_EV_IRQn: IRQn = 33;
/* !< I2C1 Event Interrupt                                              */
pub const I2C1_ER_IRQn: IRQn = 32;
/* !< TIM4 global Interrupt                                             */
pub const I2C1_EV_IRQn: IRQn = 31;
/* !< TIM3 global Interrupt                                             */
pub const TIM4_IRQn: IRQn = 30;
/* !< TIM2 global Interrupt                                             */
pub const TIM3_IRQn: IRQn = 29;
/* !< TIM1 Capture Compare Interrupt                                    */
pub const TIM2_IRQn: IRQn = 28;
/* !< TIM1 Trigger and Commutation and TIM17 Interrupt                  */
pub const TIM1_CC_IRQn: IRQn = 27;
/* !< TIM1 Update and TIM16 Interrupts                                  */
pub const TIM1_TRG_COM_TIM17_IRQn: IRQn = 26;
/* !< TIM1 Break and TIM15 Interrupts                                   */
pub const TIM1_UP_TIM16_IRQn: IRQn = 25;
/* !< External Line[9:5] Interrupts                                     */
pub const TIM1_BRK_TIM15_IRQn: IRQn = 24;
/* !< CAN1 SCE Interrupt                                                */
pub const EXTI9_5_IRQn: IRQn = 23;
/* !< CAN1 RX1 Interrupt                                                */
pub const CAN1_SCE_IRQn: IRQn = 22;
/* !< USB Device Low Priority or CAN1 RX0 Interrupts                    */
pub const CAN1_RX1_IRQn: IRQn = 21;
/* !< USB Device High Priority or CAN1 TX Interrupts                    */
pub const USB_LP_CAN1_RX0_IRQn: IRQn = 20;
/* !< ADC1 & ADC2 Interrupts                                            */
pub const USB_HP_CAN1_TX_IRQn: IRQn = 19;
/* !< DMA1 Channel 7 Interrupt                                          */
pub const ADC1_2_IRQn: IRQn = 18;
/* !< DMA1 Channel 6 Interrupt                                          */
pub const DMA1_Channel7_IRQn: IRQn = 17;
/* !< DMA1 Channel 5 Interrupt                                          */
pub const DMA1_Channel6_IRQn: IRQn = 16;
/* !< DMA1 Channel 4 Interrupt                                          */
pub const DMA1_Channel5_IRQn: IRQn = 15;
/* !< DMA1 Channel 3 Interrupt                                          */
pub const DMA1_Channel4_IRQn: IRQn = 14;
/* !< DMA1 Channel 2 Interrupt                                          */
pub const DMA1_Channel3_IRQn: IRQn = 13;
/* !< DMA1 Channel 1 Interrupt                                          */
pub const DMA1_Channel2_IRQn: IRQn = 12;
/* !< EXTI Line4 Interrupt                                              */
pub const DMA1_Channel1_IRQn: IRQn = 11;
/* !< EXTI Line3 Interrupt                                              */
pub const EXTI4_IRQn: IRQn = 10;
/* !< EXTI Line2 Interrupt and Touch Sense Interrupt                    */
pub const EXTI3_IRQn: IRQn = 9;
/* !< EXTI Line1 Interrupt                                              */
pub const EXTI2_TS_IRQn: IRQn = 8;
/* !< EXTI Line0 Interrupt                                              */
pub const EXTI1_IRQn: IRQn = 7;
/* !< RCC global Interrupt                                              */
pub const EXTI0_IRQn: IRQn = 6;
/* !< FLASH global Interrupt                                            */
pub const RCC_IRQn: IRQn = 5;
/* !< RTC Wakeup interrupt through the EXTI lines 17, 19 & 20           */
pub const FLASH_IRQn: IRQn = 4;
/* !< Tamper and TimeStamp interrupts                                   */
pub const RTC_WKUP_IRQn: IRQn = 3;
/* !< PVD through EXTI Line detection Interrupt                         */
pub const TAMPER_STAMP_IRQn: IRQn = 2;
/* !< Window WatchDog Interrupt                                         */
pub const PVD_IRQn: IRQn = 1;
/* !< 15 Cortex-M4 System Tick Interrupt                                */
/* *****  STM32 specific Interrupt Numbers **********************************************************************/
pub const WWDG_IRQn: IRQn = 0;
/* !< 14 Cortex-M4 Pend SV Interrupt                                    */
pub const SysTick_IRQn: IRQn = -1;
/* !< 12 Cortex-M4 Debug Monitor Interrupt                              */
pub const PendSV_IRQn: IRQn = -2;
/* !< 11 Cortex-M4 SV Call Interrupt                                    */
pub const DebugMonitor_IRQn: IRQn = -4;
/* !< 6 Cortex-M4 Usage Fault Interrupt                                 */
pub const SVCall_IRQn: IRQn = -5;
/* !< 5 Cortex-M4 Bus Fault Interrupt                                   */
pub const UsageFault_IRQn: IRQn = -10;
/* !< 4 Cortex-M4 Memory Management Interrupt                           */
pub const BusFault_IRQn: IRQn = -11;
/* !< 2 Non Maskable Interrupt                                          */
pub const MemoryManagement_IRQn: IRQn = -12;
/* *****  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
pub const NonMaskableInt_IRQn: IRQn = -14;
pub type IRQn_Type = IRQn;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
pub struct DMA_TypeDef {
    pub ISR: uint32_t,
    pub IFCR: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct USART_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub CR3: uint32_t,
    pub BRR: uint16_t,
    pub RESERVED1: uint16_t,
    pub GTPR: uint16_t,
    pub RESERVED2: uint16_t,
    pub RTOR: uint32_t,
    pub RQR: uint16_t,
    pub RESERVED3: uint16_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub RDR: uint16_t,
    pub RESERVED4: uint16_t,
    pub TDR: uint16_t,
    pub RESERVED5: uint16_t,
}
pub type C2RustUnnamed = libc::c_uint;
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_Speed_Level_3: C2RustUnnamed_1 = 3;
pub const GPIO_Speed_Level_2: C2RustUnnamed_1 = 2;
pub const GPIO_Speed_Level_1: C2RustUnnamed_1 = 1;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_2 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_2 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_2 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct dmaChannelDescriptor_s {
    pub dma: *mut DMA_TypeDef,
    pub ref_0: *mut DMA_Channel_TypeDef,
    pub irqHandlerCallback: dmaCallbackHandlerFuncPtr,
    pub flagsShift: uint8_t,
    pub irqN: IRQn_Type,
    pub userParam: uint32_t,
    pub owner: resourceOwner_e,
    pub resourceIndex: uint8_t,
    pub completeFlag: uint32_t,
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
pub type dmaCallbackHandlerFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut dmaChannelDescriptor_s) -> ()>;
pub type dmaChannelDescriptor_t = dmaChannelDescriptor_s;
pub type dmaIdentifier_e = libc::c_uint;
pub const DMA_LAST_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_CH5_HANDLER: dmaIdentifier_e = 12;
pub const DMA2_CH4_HANDLER: dmaIdentifier_e = 11;
pub const DMA2_CH3_HANDLER: dmaIdentifier_e = 10;
pub const DMA2_CH2_HANDLER: dmaIdentifier_e = 9;
pub const DMA2_CH1_HANDLER: dmaIdentifier_e = 8;
pub const DMA1_CH7_HANDLER: dmaIdentifier_e = 7;
pub const DMA1_CH6_HANDLER: dmaIdentifier_e = 6;
pub const DMA1_CH5_HANDLER: dmaIdentifier_e = 5;
pub const DMA1_CH4_HANDLER: dmaIdentifier_e = 4;
pub const DMA1_CH3_HANDLER: dmaIdentifier_e = 3;
pub const DMA1_CH2_HANDLER: dmaIdentifier_e = 2;
pub const DMA1_CH1_HANDLER: dmaIdentifier_e = 1;
pub const DMA_NONE: dmaIdentifier_e = 0;
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
pub type rcc_reg = libc::c_uint;
pub const RCC_AHB1: rcc_reg = 4;
pub const RCC_APB1: rcc_reg = 3;
pub const RCC_APB2: rcc_reg = 2;
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
pub const RCC_EMPTY: rcc_reg = 0;
pub type portMode_e = libc::c_uint;
pub const MODE_RXTX: portMode_e = 3;
pub const MODE_TX: portMode_e = 2;
pub const MODE_RX: portMode_e = 1;
pub type portOptions_e = libc::c_uint;
pub const SERIAL_BIDIR_NOPULL: portOptions_e = 32;
pub const SERIAL_BIDIR_PP: portOptions_e = 16;
pub const SERIAL_BIDIR_OD: portOptions_e = 0;
pub const SERIAL_BIDIR: portOptions_e = 8;
pub const SERIAL_UNIDIR: portOptions_e = 0;
pub const SERIAL_PARITY_EVEN: portOptions_e = 4;
pub const SERIAL_PARITY_NO: portOptions_e = 0;
pub const SERIAL_STOPBITS_2: portOptions_e = 2;
pub const SERIAL_STOPBITS_1: portOptions_e = 0;
pub const SERIAL_INVERTED: portOptions_e = 1;
pub const SERIAL_NOT_INVERTED: portOptions_e = 0;
// Define known line control states which may be passed up by underlying serial driver callback
pub type serialReceiveCallbackPtr
    =
    Option<unsafe extern "C" fn(_: uint16_t, _: *mut libc::c_void) -> ()>;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPort_s {
    pub vTable: *const serialPortVTable,
    pub mode: portMode_e,
    pub options: portOptions_e,
    pub baudRate: uint32_t,
    pub rxBufferSize: uint32_t,
    pub txBufferSize: uint32_t,
    pub rxBuffer: *mut uint8_t,
    pub txBuffer: *mut uint8_t,
    pub rxBufferHead: uint32_t,
    pub rxBufferTail: uint32_t,
    pub txBufferHead: uint32_t,
    pub txBufferTail: uint32_t,
    pub rxCallback: serialReceiveCallbackPtr,
    pub rxCallbackData: *mut libc::c_void,
    pub identifier: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct serialPortVTable {
    pub serialWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                 _: uint8_t) -> ()>,
    pub serialTotalRxWaiting: Option<unsafe extern "C" fn(_:
                                                              *const serialPort_t)
                                         -> uint32_t>,
    pub serialTotalTxFree: Option<unsafe extern "C" fn(_: *const serialPort_t)
                                      -> uint32_t>,
    pub serialRead: Option<unsafe extern "C" fn(_: *mut serialPort_t)
                               -> uint8_t>,
    pub serialSetBaudRate: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                       _: uint32_t) -> ()>,
    pub isSerialTransmitBufferEmpty: Option<unsafe extern "C" fn(_:
                                                                     *const serialPort_t)
                                                -> bool>,
    pub setMode: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                             _: portMode_e) -> ()>,
    pub setCtrlLineStateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                        _:
                                                            Option<unsafe extern "C" fn(_:
                                                                                            *mut libc::c_void,
                                                                                        _:
                                                                                            uint16_t)
                                                                       -> ()>,
                                                        _: *mut libc::c_void)
                                       -> ()>,
    pub setBaudRateCb: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                                   _:
                                                       Option<unsafe extern "C" fn(_:
                                                                                       *mut serialPort_t,
                                                                                   _:
                                                                                       uint32_t)
                                                                  -> ()>,
                                                   _: *mut serialPort_t)
                                  -> ()>,
    pub writeBuf: Option<unsafe extern "C" fn(_: *mut serialPort_t,
                                              _: *const libc::c_void,
                                              _: libc::c_int) -> ()>,
    pub beginWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
    pub endWrite: Option<unsafe extern "C" fn(_: *mut serialPort_t) -> ()>,
}
// used by serial drivers to return frames to app
pub type serialPort_t = serialPort_s;
pub type UARTDevice_e = libc::c_uint;
pub const UARTDEV_8: UARTDevice_e = 7;
pub const UARTDEV_7: UARTDevice_e = 6;
pub const UARTDEV_6: UARTDevice_e = 5;
pub const UARTDEV_5: UARTDevice_e = 4;
pub const UARTDEV_4: UARTDevice_e = 3;
pub const UARTDEV_3: UARTDevice_e = 2;
pub const UARTDEV_2: UARTDevice_e = 1;
pub const UARTDEV_1: UARTDevice_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartPort_s {
    pub port: serialPort_t,
    pub rxDMAChannel: *mut DMA_Channel_TypeDef,
    pub txDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxDMAIrq: uint32_t,
    pub txDMAIrq: uint32_t,
    pub rxDMAPos: uint32_t,
    pub txDMAPeripheralBaseAddr: uint32_t,
    pub rxDMAPeripheralBaseAddr: uint32_t,
    pub USARTx: *mut USART_TypeDef,
    pub txDMAEmpty: bool,
}
pub type uartPort_t = uartPort_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartHardware_s {
    pub device: UARTDevice_e,
    pub reg: *mut USART_TypeDef,
    pub txDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxDMAChannel: *mut DMA_Channel_TypeDef,
    pub rxPins: [ioTag_t; 4],
    pub txPins: [ioTag_t; 4],
    pub rcc: rccPeriphTag_t,
    pub af: uint8_t,
    pub irqn: uint8_t,
    pub txPriority: uint8_t,
    pub rxPriority: uint8_t,
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
// Configuration constants
// Count number of configured UARTs
pub type uartHardware_t = uartHardware_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct uartDevice_s {
    pub port: uartPort_t,
    pub hardware: *const uartHardware_t,
    pub rx: ioTag_t,
    pub tx: ioTag_t,
    pub rxBuffer: [uint8_t; 128],
    pub txBuffer: [uint8_t; 256],
}
// XXX Not required for full allocation
// uartDevice_t is an actual device instance.
// XXX Instances are allocated for uarts defined by USE_UARTx atm.
pub type uartDevice_t = uartDevice_s;
// Initialized in run_static_initializers
#[no_mangle]
pub static mut uartHardware: [uartHardware_t; 1] =
    [uartHardware_t{device: UARTDEV_1,
                    reg: 0 as *mut USART_TypeDef,
                    txDMAChannel: 0 as *mut DMA_Channel_TypeDef,
                    rxDMAChannel: 0 as *mut DMA_Channel_TypeDef,
                    rxPins: [0; 4],
                    txPins: [0; 4],
                    rcc: 0,
                    af: 0,
                    irqn: 0,
                    txPriority: 0,
                    rxPriority: 0,}; 1];
unsafe extern "C" fn handleUsartTxDma(mut descriptor:
                                          *mut dmaChannelDescriptor_t) {
    let mut s: *mut uartPort_t = (*descriptor).userParam as *mut uartPort_t;
    ::core::ptr::write_volatile(&mut (*(*descriptor).dma).IFCR as
                                    *mut uint32_t,
                                (0x2 as libc::c_int as uint32_t) <<
                                    (*descriptor).flagsShift as libc::c_int);
    DMA_Cmd((*descriptor).ref_0, DISABLE);
    uartTryStartTxDMA(s);
}
#[no_mangle]
pub unsafe extern "C" fn serialUARTInitIO(mut txIO: IO_t, mut rxIO: IO_t,
                                          mut mode: portMode_e,
                                          mut options: portOptions_e,
                                          mut af: uint8_t,
                                          mut index: uint8_t) {
    if options as libc::c_uint & SERIAL_BIDIR as libc::c_int as libc::c_uint
           != 0 && !txIO.is_null() {
        let mut ioCfg: ioConfig_t =
            (GPIO_Mode_AF as libc::c_int |
                 (GPIO_Speed_Level_3 as libc::c_int) << 2 as libc::c_int |
                 (if options as libc::c_uint &
                         SERIAL_INVERTED as libc::c_int as libc::c_uint != 0
                         ||
                         options as libc::c_uint &
                             SERIAL_BIDIR_PP as libc::c_int as libc::c_uint !=
                             0 {
                      GPIO_OType_PP as libc::c_int
                  } else { GPIO_OType_OD as libc::c_int }) << 4 as libc::c_int
                 |
                 (if options as libc::c_uint &
                         SERIAL_INVERTED as libc::c_int as libc::c_uint != 0
                         ||
                         options as libc::c_uint &
                             SERIAL_BIDIR_PP as libc::c_int as libc::c_uint !=
                             0 {
                      GPIO_PuPd_DOWN as libc::c_int
                  } else { GPIO_PuPd_UP as libc::c_int }) << 5 as libc::c_int)
                as ioConfig_t;
        IOInit(txIO, OWNER_SERIAL_TX,
               (index as libc::c_int + 1 as libc::c_int) as uint8_t);
        IOConfigGPIOAF(txIO, ioCfg, af);
        if options as libc::c_uint &
               SERIAL_INVERTED as libc::c_int as libc::c_uint == 0 {
            IOLo(txIO);
        }
        // OpenDrain output should be inactive
    } else {
        let mut ioCfg_0: ioConfig_t =
            (GPIO_Mode_AF as libc::c_int |
                 (GPIO_Speed_Level_3 as libc::c_int) << 2 as libc::c_int |
                 (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                 (if options as libc::c_uint &
                         SERIAL_INVERTED as libc::c_int as libc::c_uint != 0 {
                      GPIO_PuPd_DOWN as libc::c_int
                  } else { GPIO_PuPd_UP as libc::c_int }) << 5 as libc::c_int)
                as ioConfig_t;
        if mode as libc::c_uint & MODE_TX as libc::c_int as libc::c_uint != 0
               && !txIO.is_null() {
            IOInit(txIO, OWNER_SERIAL_TX,
                   (index as libc::c_int + 1 as libc::c_int) as uint8_t);
            IOConfigGPIOAF(txIO, ioCfg_0, af);
        }
        if mode as libc::c_uint & MODE_RX as libc::c_int as libc::c_uint != 0
               && !rxIO.is_null() {
            IOInit(rxIO, OWNER_SERIAL_RX,
                   (index as libc::c_int + 1 as libc::c_int) as uint8_t);
            IOConfigGPIOAF(rxIO, ioCfg_0, af);
        }
    };
}
// XXX Should serialUART be consolidated?
#[no_mangle]
pub unsafe extern "C" fn serialUART(mut device: UARTDevice_e,
                                    mut baudRate: uint32_t,
                                    mut mode: portMode_e,
                                    mut options: portOptions_e)
 -> *mut uartPort_t {
    let mut uartDev: *mut uartDevice_t =
        *uartDevmap.as_mut_ptr().offset(device as isize);
    if uartDev.is_null() { return 0 as *mut uartPort_t }
    let mut s: *mut uartPort_t = &mut (*uartDev).port;
    (*s).port.vTable = uartVTable.as_ptr();
    (*s).port.baudRate = baudRate;
    (*s).port.rxBuffer = (*uartDev).rxBuffer.as_mut_ptr();
    (*s).port.txBuffer = (*uartDev).txBuffer.as_mut_ptr();
    (*s).port.rxBufferSize =
        ::core::mem::size_of::<[uint8_t; 128]>() as libc::c_ulong as uint32_t;
    (*s).port.txBufferSize =
        ::core::mem::size_of::<[uint8_t; 256]>() as libc::c_ulong as uint32_t;
    let mut hardware: *const uartHardware_t = (*uartDev).hardware;
    (*s).USARTx = (*hardware).reg;
    RCC_ClockCmd((*hardware).rcc, ENABLE);
    if !(*hardware).rxDMAChannel.is_null() {
        dmaInit(dmaGetIdentifier((*hardware).rxDMAChannel), OWNER_SERIAL_RX,
                (device as
                     libc::c_uint).wrapping_add(1 as libc::c_int as
                                                    libc::c_uint) as uint8_t);
        (*s).rxDMAChannel = (*hardware).rxDMAChannel;
        (*s).rxDMAPeripheralBaseAddr =
            &mut (*(*s).USARTx).RDR as *mut uint16_t as uint32_t
    }
    if !(*hardware).txDMAChannel.is_null() {
        let identifier: dmaIdentifier_e =
            dmaGetIdentifier((*hardware).txDMAChannel);
        dmaInit(identifier, OWNER_SERIAL_TX,
                (device as
                     libc::c_uint).wrapping_add(1 as libc::c_int as
                                                    libc::c_uint) as uint8_t);
        dmaSetHandler(identifier,
                      Some(handleUsartTxDma as
                               unsafe extern "C" fn(_:
                                                        *mut dmaChannelDescriptor_t)
                                   -> ()), (*hardware).txPriority as uint32_t,
                      s as uint32_t);
        (*s).txDMAChannel = (*hardware).txDMAChannel;
        (*s).txDMAPeripheralBaseAddr =
            &mut (*(*s).USARTx).TDR as *mut uint16_t as uint32_t
    }
    serialUARTInitIO(IOGetByTag((*uartDev).tx), IOGetByTag((*uartDev).rx),
                     mode, options, (*hardware).af, device as uint8_t);
    if (*s).rxDMAChannel.is_null() || (*s).txDMAChannel.is_null() {
        let mut NVIC_InitStructure: NVIC_InitTypeDef =
            NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                             NVIC_IRQChannelPreemptionPriority: 0,
                             NVIC_IRQChannelSubPriority: 0,
                             NVIC_IRQChannelCmd: DISABLE,};
        NVIC_InitStructure.NVIC_IRQChannel = (*hardware).irqn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
            ((*hardware).rxPriority as libc::c_int >>
                 (4 as libc::c_int as
                      libc::c_uint).wrapping_sub((7 as libc::c_int as
                                                      libc::c_uint).wrapping_sub(0x500
                                                                                     as
                                                                                     libc::c_int
                                                                                     as
                                                                                     uint32_t
                                                                                     >>
                                                                                     8
                                                                                         as
                                                                                         libc::c_int))
                 >> 4 as libc::c_int) as uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority =
            (((*hardware).rxPriority as libc::c_int &
                  0xf as libc::c_int >>
                      (7 as libc::c_int as
                           libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                          uint32_t >>
                                                          8 as libc::c_int))
                 >> 4 as libc::c_int) as uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&mut NVIC_InitStructure);
    }
    return s;
}
#[no_mangle]
pub unsafe extern "C" fn uartIrqHandler(mut s: *mut uartPort_t) {
    let mut ISR: uint32_t = (*(*s).USARTx).ISR;
    if (*s).rxDMAChannel.is_null() &&
           ISR & 0x20 as libc::c_int as uint32_t != 0 {
        if (*s).port.rxCallback.is_some() {
            (*s).port.rxCallback.expect("non-null function pointer")((*(*s).USARTx).RDR,
                                                                     (*s).port.rxCallbackData);
        } else {
            let fresh0 = (*s).port.rxBufferHead;
            (*s).port.rxBufferHead = (*s).port.rxBufferHead.wrapping_add(1);
            ::core::ptr::write_volatile((*s).port.rxBuffer.offset(fresh0 as
                                                                      isize),
                                        (*(*s).USARTx).RDR as uint8_t);
            if (*s).port.rxBufferHead >= (*s).port.rxBufferSize {
                (*s).port.rxBufferHead = 0 as libc::c_int as uint32_t
            }
        }
    }
    if (*s).txDMAChannel.is_null() &&
           ISR & 0x80 as libc::c_int as uint32_t != 0 {
        if (*s).port.txBufferTail != (*s).port.txBufferHead {
            let fresh1 = (*s).port.txBufferTail;
            (*s).port.txBufferTail = (*s).port.txBufferTail.wrapping_add(1);
            USART_SendData((*s).USARTx,
                           *(*s).port.txBuffer.offset(fresh1 as isize) as
                               uint16_t);
            if (*s).port.txBufferTail >= (*s).port.txBufferSize {
                (*s).port.txBufferTail = 0 as libc::c_int as uint32_t
            }
        } else {
            USART_ITConfig((*s).USARTx, 0x70107 as libc::c_int as uint32_t,
                           DISABLE);
        }
    }
    if ISR & 0x8 as libc::c_int as uint32_t != 0 {
        USART_ClearITPendingBit((*s).USARTx,
                                0x30300 as libc::c_int as uint32_t);
    };
}
unsafe extern "C" fn run_static_initializers() {
    uartHardware =
        [{
             let mut init =
                 uartHardware_s{device: UARTDEV_3,
                                reg:
                                    (0x40000000 as libc::c_int as
                                         uint32_t).wrapping_add(0x4800 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_uint)
                                        as *mut USART_TypeDef,
                                txDMAChannel: 0 as *mut DMA_Channel_TypeDef,
                                rxDMAChannel: 0 as *mut DMA_Channel_TypeDef,
                                rxPins:
                                    [((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          11 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                txPins:
                                    [((1 as libc::c_int + 1 as libc::c_int) <<
                                          4 as libc::c_int |
                                          10 as libc::c_int) as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t,
                                     0 as libc::c_int as ioTag_t],
                                rcc:
                                    (((RCC_APB1 as libc::c_int) <<
                                          5 as libc::c_int) as libc::c_long |
                                         (16 as libc::c_int *
                                              (0x40000 as libc::c_int as
                                                   uint32_t as libc::c_long >
                                                   65535 as libc::c_long) as
                                                  libc::c_int) as libc::c_long
                                             +
                                             ((8 as libc::c_int *
                                                   (0x40000 as libc::c_int as
                                                        uint32_t as
                                                        libc::c_long *
                                                        1 as libc::c_long >>
                                                        16 as libc::c_int *
                                                            (0x40000 as
                                                                 libc::c_int
                                                                 as uint32_t
                                                                 as
                                                                 libc::c_long
                                                                 >
                                                                 65535 as
                                                                     libc::c_long)
                                                                as libc::c_int
                                                        >
                                                        255 as libc::c_int as
                                                            libc::c_long) as
                                                       libc::c_int) as
                                                  libc::c_long +
                                                  (8 as libc::c_int as
                                                       libc::c_long -
                                                       90 as libc::c_int as
                                                           libc::c_long /
                                                           ((0x40000 as
                                                                 libc::c_int
                                                                 as uint32_t
                                                                 as
                                                                 libc::c_long
                                                                 *
                                                                 1 as
                                                                     libc::c_long
                                                                 >>
                                                                 16 as
                                                                     libc::c_int
                                                                     *
                                                                     (0x40000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          uint32_t
                                                                          as
                                                                          libc::c_long
                                                                          >
                                                                          65535
                                                                              as
                                                                              libc::c_long)
                                                                         as
                                                                         libc::c_int
                                                                 >>
                                                                 8 as
                                                                     libc::c_int
                                                                     *
                                                                     (0x40000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          uint32_t
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (0x40000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint32_t
                                                                                   as
                                                                                   libc::c_long
                                                                                   >
                                                                                   65535
                                                                                       as
                                                                                       libc::c_long)
                                                                                  as
                                                                                  libc::c_int
                                                                          >
                                                                          255
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_long)
                                                                         as
                                                                         libc::c_int)
                                                                /
                                                                4 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_long
                                                                +
                                                                14 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_long
                                                                |
                                                                1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_long)
                                                       -
                                                       2 as libc::c_int as
                                                           libc::c_long /
                                                           ((0x40000 as
                                                                 libc::c_int
                                                                 as uint32_t
                                                                 as
                                                                 libc::c_long
                                                                 *
                                                                 1 as
                                                                     libc::c_long
                                                                 >>
                                                                 16 as
                                                                     libc::c_int
                                                                     *
                                                                     (0x40000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          uint32_t
                                                                          as
                                                                          libc::c_long
                                                                          >
                                                                          65535
                                                                              as
                                                                              libc::c_long)
                                                                         as
                                                                         libc::c_int
                                                                 >>
                                                                 8 as
                                                                     libc::c_int
                                                                     *
                                                                     (0x40000
                                                                          as
                                                                          libc::c_int
                                                                          as
                                                                          uint32_t
                                                                          as
                                                                          libc::c_long
                                                                          *
                                                                          1 as
                                                                              libc::c_long
                                                                          >>
                                                                          16
                                                                              as
                                                                              libc::c_int
                                                                              *
                                                                              (0x40000
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   uint32_t
                                                                                   as
                                                                                   libc::c_long
                                                                                   >
                                                                                   65535
                                                                                       as
                                                                                       libc::c_long)
                                                                                  as
                                                                                  libc::c_int
                                                                          >
                                                                          255
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_long)
                                                                         as
                                                                         libc::c_int)
                                                                /
                                                                2 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_long
                                                                +
                                                                1 as
                                                                    libc::c_int
                                                                    as
                                                                    libc::c_long))))
                                        as rccPeriphTag_t,
                                af: 0x7 as libc::c_int as uint8_t,
                                irqn: USART3_IRQn as libc::c_int as uint8_t,
                                txPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x500
                                                                                                              as
                                                                                                              libc::c_int
                                                                                                              as
                                                                                                              uint32_t
                                                                                                              >>
                                                                                                              8
                                                                                                                  as
                                                                                                                  libc::c_int))
                                          |
                                          0 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x500
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      uint32_t
                                                                                      >>
                                                                                      8
                                                                                          as
                                                                                          libc::c_int))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,
                                rxPriority:
                                    (((1 as libc::c_int) <<
                                          (4 as libc::c_int as
                                               libc::c_uint).wrapping_sub((7
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint).wrapping_sub(0x500
                                                                                                              as
                                                                                                              libc::c_int
                                                                                                              as
                                                                                                              uint32_t
                                                                                                              >>
                                                                                                              8
                                                                                                                  as
                                                                                                                  libc::c_int))
                                          |
                                          1 as libc::c_int &
                                              0xf as libc::c_int >>
                                                  (7 as libc::c_int as
                                                       libc::c_uint).wrapping_sub(0x500
                                                                                      as
                                                                                      libc::c_int
                                                                                      as
                                                                                      uint32_t
                                                                                      >>
                                                                                      8
                                                                                          as
                                                                                          libc::c_int))
                                         << 4 as libc::c_int &
                                         0xf0 as libc::c_int) as uint8_t,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
// USE_UART
