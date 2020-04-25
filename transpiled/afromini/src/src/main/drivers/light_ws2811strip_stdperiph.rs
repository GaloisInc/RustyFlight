use core;
use libc;
extern "C" {
    /* *
  * @}
  */
    /* * @defgroup DMA_flags_definition 
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
    /* Function used to set the DMA configuration to the default reset state ******/
    #[no_mangle]
    fn DMA_DeInit(DMAy_Channelx: *mut DMA_Channel_TypeDef);
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn DMA_Init(DMAy_Channelx: *mut DMA_Channel_TypeDef,
                DMA_InitStruct: *mut DMA_InitTypeDef);
    #[no_mangle]
    fn DMA_StructInit(DMA_InitStruct: *mut DMA_InitTypeDef);
    #[no_mangle]
    fn DMA_Cmd(DMAy_Channelx: *mut DMA_Channel_TypeDef,
               NewState: FunctionalState);
    /* Data Counter functions******************************************************/
    #[no_mangle]
    fn DMA_SetCurrDataCounter(DMAy_Channelx: *mut DMA_Channel_TypeDef,
                              DataNumber: uint16_t);
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn DMA_ITConfig(DMAy_Channelx: *mut DMA_Channel_TypeDef, DMA_IT: uint32_t,
                    NewState: FunctionalState);
    #[no_mangle]
    fn TIM_TimeBaseInit(TIMx: *mut TIM_TypeDef,
                        TIM_TimeBaseInitStruct: *mut TIM_TimeBaseInitTypeDef);
    #[no_mangle]
    fn TIM_TimeBaseStructInit(TIM_TimeBaseInitStruct:
                                  *mut TIM_TimeBaseInitTypeDef);
    #[no_mangle]
    fn TIM_SetCounter(TIMx: *mut TIM_TypeDef, Counter: uint32_t);
    #[no_mangle]
    fn TIM_ARRPreloadConfig(TIMx: *mut TIM_TypeDef,
                            NewState: FunctionalState);
    #[no_mangle]
    fn TIM_Cmd(TIMx: *mut TIM_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn TIM_OCStructInit(TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_CCxCmd(TIMx: *mut TIM_TypeDef, TIM_Channel: uint16_t,
                  TIM_CCx: uint16_t);
    #[no_mangle]
    fn TIM_CCxNCmd(TIMx: *mut TIM_TypeDef, TIM_Channel: uint16_t,
                   TIM_CCxN: uint16_t);
    #[no_mangle]
    fn TIM_CtrlPWMOutputs(TIMx: *mut TIM_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn TIM_DMACmd(TIMx: *mut TIM_TypeDef, TIM_DMASource: uint16_t,
                  NewState: FunctionalState);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    static mut BIT_COMPARE_1: uint16_t;
    #[no_mangle]
    static mut BIT_COMPARE_0: uint16_t;
    #[no_mangle]
    static mut ws2811LedDataTransferInProgress: uint8_t;
    #[no_mangle]
    static mut ledStripDMABuffer: [uint8_t; 810];
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    // TODO - just for migration
    #[no_mangle]
    fn timerRCC(tim: *mut TIM_TypeDef) -> rccPeriphTag_t;
    #[no_mangle]
    fn timerGetByTag(ioTag: ioTag_t) -> *const timerHardware_t;
    #[no_mangle]
    fn timerOCInit(tim: *mut TIM_TypeDef, channel: uint8_t,
                   init: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn timerOCPreloadConfig(tim: *mut TIM_TypeDef, channel: uint8_t,
                            preload: uint16_t);
    #[no_mangle]
    fn timerCCR(tim: *mut TIM_TypeDef, channel: uint8_t) -> *mut timCCR_t;
    #[no_mangle]
    fn timerDmaSource(channel: uint8_t) -> uint16_t;
    #[no_mangle]
    fn timerGetPrescalerByDesiredMhz(tim: *mut TIM_TypeDef, mhz: uint16_t)
     -> uint16_t;
    #[no_mangle]
    fn timerGetPeriodByPrescaler(tim: *mut TIM_TypeDef, prescaler: uint16_t,
                                 hz: uint32_t) -> uint16_t;
}
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
pub type IRQn = libc::c_int;
pub const FPU_IRQn: IRQn = 81;
pub const USBWakeUp_RMP_IRQn: IRQn = 76;
pub const USB_LP_IRQn: IRQn = 75;
pub const USB_HP_IRQn: IRQn = 74;
pub const COMP7_IRQn: IRQn = 66;
pub const COMP4_5_6_IRQn: IRQn = 65;
pub const COMP1_2_3_IRQn: IRQn = 64;
pub const ADC4_IRQn: IRQn = 61;
pub const DMA2_Channel5_IRQn: IRQn = 60;
pub const DMA2_Channel4_IRQn: IRQn = 59;
pub const DMA2_Channel3_IRQn: IRQn = 58;
pub const DMA2_Channel2_IRQn: IRQn = 57;
pub const DMA2_Channel1_IRQn: IRQn = 56;
pub const TIM7_IRQn: IRQn = 55;
pub const TIM6_DAC_IRQn: IRQn = 54;
pub const UART5_IRQn: IRQn = 53;
pub const UART4_IRQn: IRQn = 52;
pub const SPI3_IRQn: IRQn = 51;
pub const ADC3_IRQn: IRQn = 47;
pub const TIM8_CC_IRQn: IRQn = 46;
pub const TIM8_TRG_COM_IRQn: IRQn = 45;
pub const TIM8_UP_IRQn: IRQn = 44;
pub const TIM8_BRK_IRQn: IRQn = 43;
pub const USBWakeUp_IRQn: IRQn = 42;
pub const RTC_Alarm_IRQn: IRQn = 41;
pub const EXTI15_10_IRQn: IRQn = 40;
pub const USART3_IRQn: IRQn = 39;
pub const USART2_IRQn: IRQn = 38;
pub const USART1_IRQn: IRQn = 37;
pub const SPI2_IRQn: IRQn = 36;
pub const SPI1_IRQn: IRQn = 35;
pub const I2C2_ER_IRQn: IRQn = 34;
pub const I2C2_EV_IRQn: IRQn = 33;
pub const I2C1_ER_IRQn: IRQn = 32;
pub const I2C1_EV_IRQn: IRQn = 31;
pub const TIM4_IRQn: IRQn = 30;
pub const TIM3_IRQn: IRQn = 29;
pub const TIM2_IRQn: IRQn = 28;
pub const TIM1_CC_IRQn: IRQn = 27;
pub const TIM1_TRG_COM_TIM17_IRQn: IRQn = 26;
pub const TIM1_UP_TIM16_IRQn: IRQn = 25;
pub const TIM1_BRK_TIM15_IRQn: IRQn = 24;
pub const EXTI9_5_IRQn: IRQn = 23;
pub const CAN1_SCE_IRQn: IRQn = 22;
pub const CAN1_RX1_IRQn: IRQn = 21;
pub const USB_LP_CAN1_RX0_IRQn: IRQn = 20;
pub const USB_HP_CAN1_TX_IRQn: IRQn = 19;
pub const ADC1_2_IRQn: IRQn = 18;
pub const DMA1_Channel7_IRQn: IRQn = 17;
pub const DMA1_Channel6_IRQn: IRQn = 16;
pub const DMA1_Channel5_IRQn: IRQn = 15;
pub const DMA1_Channel4_IRQn: IRQn = 14;
pub const DMA1_Channel3_IRQn: IRQn = 13;
pub const DMA1_Channel2_IRQn: IRQn = 12;
pub const DMA1_Channel1_IRQn: IRQn = 11;
pub const EXTI4_IRQn: IRQn = 10;
pub const EXTI3_IRQn: IRQn = 9;
pub const EXTI2_TS_IRQn: IRQn = 8;
pub const EXTI1_IRQn: IRQn = 7;
pub const EXTI0_IRQn: IRQn = 6;
pub const RCC_IRQn: IRQn = 5;
pub const FLASH_IRQn: IRQn = 4;
pub const RTC_WKUP_IRQn: IRQn = 3;
pub const TAMPER_STAMP_IRQn: IRQn = 2;
pub const PVD_IRQn: IRQn = 1;
pub const WWDG_IRQn: IRQn = 0;
pub const SysTick_IRQn: IRQn = -1;
pub const PendSV_IRQn: IRQn = -2;
pub const DebugMonitor_IRQn: IRQn = -4;
pub const SVCall_IRQn: IRQn = -5;
pub const UsageFault_IRQn: IRQn = -10;
pub const BusFault_IRQn: IRQn = -11;
pub const MemoryManagement_IRQn: IRQn = -12;
pub const NonMaskableInt_IRQn: IRQn = -14;
pub type IRQn_Type = IRQn;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_Channel_TypeDef {
    pub CCR: uint32_t,
    pub CNDTR: uint32_t,
    pub CPAR: uint32_t,
    pub CMAR: uint32_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_TypeDef {
    pub ISR: uint32_t,
    pub IFCR: uint32_t,
}
#[derive ( Copy, Clone )]
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct DMA_InitTypeDef {
    pub DMA_PeripheralBaseAddr: uint32_t,
    pub DMA_MemoryBaseAddr: uint32_t,
    pub DMA_DIR: uint32_t,
    pub DMA_BufferSize: uint16_t,
    pub DMA_PeripheralInc: uint32_t,
    pub DMA_MemoryInc: uint32_t,
    pub DMA_PeripheralDataSize: uint32_t,
    pub DMA_MemoryDataSize: uint32_t,
    pub DMA_Mode: uint32_t,
    pub DMA_Priority: uint32_t,
    pub DMA_M2M: uint32_t,
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
pub type C2RustUnnamed = libc::c_uint;
/* !< GPIO Analog In/Out Mode      */
/* !< GPIO Alternate function Mode */
pub const GPIO_Mode_AN: C2RustUnnamed = 3;
/* !< GPIO Output Mode */
pub const GPIO_Mode_AF: C2RustUnnamed = 2;
/* !< GPIO Input Mode */
pub const GPIO_Mode_OUT: C2RustUnnamed = 1;
pub const GPIO_Mode_IN: C2RustUnnamed = 0;
/* *
  * @}
  */
/* * @defgroup Output_type_enumeration
  * @{
  */
pub type C2RustUnnamed_0 = libc::c_uint;
pub const GPIO_OType_OD: C2RustUnnamed_0 = 1;
pub const GPIO_OType_PP: C2RustUnnamed_0 = 0;
/* *
  * @}
  */
/* * @defgroup Output_Maximum_frequency_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
/* !< High Speed     */
/* !< Meduim Speed   */
pub const GPIO_Speed_Level_3: C2RustUnnamed_1 = 3;
/* !< Fast Speed     */
pub const GPIO_Speed_Level_2: C2RustUnnamed_1 = 2;
pub const GPIO_Speed_Level_1: C2RustUnnamed_1 = 1;
/* *
  * @}
  */
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_2 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_2 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_2 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_2 = 0;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_TimeBaseInitTypeDef {
    pub TIM_Prescaler: uint16_t,
    pub TIM_CounterMode: uint16_t,
    pub TIM_Period: uint32_t,
    pub TIM_ClockDivision: uint16_t,
    pub TIM_RepetitionCounter: uint16_t,
}
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct TIM_OCInitTypeDef {
    pub TIM_OCMode: uint32_t,
    pub TIM_OutputState: uint16_t,
    pub TIM_OutputNState: uint16_t,
    pub TIM_Pulse: uint32_t,
    pub TIM_OCPolarity: uint16_t,
    pub TIM_OCNPolarity: uint16_t,
    pub TIM_OCIdleState: uint16_t,
    pub TIM_OCNIdleState: uint16_t,
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
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct hsvColor_s {
    pub h: uint16_t,
    pub s: uint8_t,
    pub v: uint8_t,
}
pub type hsvColor_t = hsvColor_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct modeColorIndexes_s {
    pub color: [uint8_t; 6],
}
pub type modeColorIndexes_t = modeColorIndexes_s;
#[derive ( Copy, Clone )]
#[repr(C)]
pub struct specialColorIndexes_s {
    pub color: [uint8_t; 11],
}
pub type specialColorIndexes_t = specialColorIndexes_s;
pub type timerHardware_t = timerHardware_s;
#[derive ( Copy, Clone )]
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
// 0 - 359
// 0 - 255
// 0 - 255
// TIMUP
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type dmaChannelDescriptor_t = dmaChannelDescriptor_s;
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
#[derive ( Copy, Clone )]
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
pub type dmaCallbackHandlerFuncPtr
    =
    Option<unsafe extern "C" fn(_: *mut dmaChannelDescriptor_s) -> ()>;
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
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed_3 = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed_3 = 1;
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed_3 = 0;
// suppress LEDLOW mode if beeper is on
#[no_mangle]
pub static mut specialColors: specialColorIndexes_t =
    specialColorIndexes_t{color: [0; 11],};
#[no_mangle]
pub static mut modeColors: *const modeColorIndexes_t =
    0 as *const modeColorIndexes_t;
#[no_mangle]
pub static mut colors: *mut hsvColor_t =
    0 as *const hsvColor_t as *mut hsvColor_t;
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
static mut ws2811IO: IO_t = 0 as *const libc::c_void as IO_t;
#[no_mangle]
pub static mut ws2811Initialised: bool = 0i32 != 0;
static mut dmaRef: *mut DMA_Channel_TypeDef =
    0 as *const DMA_Channel_TypeDef as *mut DMA_Channel_TypeDef;
static mut timer: *mut TIM_TypeDef =
    0 as *const TIM_TypeDef as *mut TIM_TypeDef;
unsafe extern "C" fn WS2811_DMA_IRQHandler(mut descriptor:
                                               *mut dmaChannelDescriptor_t) {
    if (*(*descriptor).dma).ISR &
           (0x2i32 as uint32_t) << (*descriptor).flagsShift as libc::c_int !=
           0 {
        ::core::ptr::write_volatile(&mut ws2811LedDataTransferInProgress as
                                        *mut uint8_t, 0i32 as uint8_t);
        DMA_Cmd((*descriptor).ref_0, DISABLE);
        ::core::ptr::write_volatile(&mut (*(*descriptor).dma).IFCR as
                                        *mut uint32_t,
                                    (0x2i32 as uint32_t) <<
                                        (*descriptor).flagsShift as
                                            libc::c_int)
    };
}
#[no_mangle]
pub unsafe extern "C" fn ws2811LedStripHardwareInit(mut ioTag: ioTag_t) {
    if ioTag == 0 { return }
    let mut TIM_TimeBaseStructure: TIM_TimeBaseInitTypeDef =
        TIM_TimeBaseInitTypeDef{TIM_Prescaler: 0,
                                TIM_CounterMode: 0,
                                TIM_Period: 0,
                                TIM_ClockDivision: 0,
                                TIM_RepetitionCounter: 0,};
    let mut TIM_OCInitStructure: TIM_OCInitTypeDef =
        TIM_OCInitTypeDef{TIM_OCMode: 0,
                          TIM_OutputState: 0,
                          TIM_OutputNState: 0,
                          TIM_Pulse: 0,
                          TIM_OCPolarity: 0,
                          TIM_OCNPolarity: 0,
                          TIM_OCIdleState: 0,
                          TIM_OCNIdleState: 0,};
    let mut DMA_InitStructure: DMA_InitTypeDef =
        DMA_InitTypeDef{DMA_PeripheralBaseAddr: 0,
                        DMA_MemoryBaseAddr: 0,
                        DMA_DIR: 0,
                        DMA_BufferSize: 0,
                        DMA_PeripheralInc: 0,
                        DMA_MemoryInc: 0,
                        DMA_PeripheralDataSize: 0,
                        DMA_MemoryDataSize: 0,
                        DMA_Mode: 0,
                        DMA_Priority: 0,
                        DMA_M2M: 0,};
    let mut timerHardware: *const timerHardware_t = timerGetByTag(ioTag);
    timer = (*timerHardware).tim;
    if (*timerHardware).dmaRef.is_null() { return }
    ws2811IO = IOGetByTag(ioTag);
    IOInit(ws2811IO, OWNER_LED_STRIP, 0i32 as uint8_t);
    IOConfigGPIOAF(ws2811IO,
                   (GPIO_Mode_AF as libc::c_int |
                        (GPIO_Speed_Level_3 as libc::c_int) << 2i32 |
                        (GPIO_OType_PP as libc::c_int) << 4i32 |
                        (GPIO_PuPd_UP as libc::c_int) << 5i32) as ioConfig_t,
                   (*timerHardware).alternateFunction);
    RCC_ClockCmd(timerRCC(timer), ENABLE);
    // Stop timer
    TIM_Cmd(timer, DISABLE);
    /* Compute the prescaler value */
    let mut prescaler: uint16_t =
        timerGetPrescalerByDesiredMhz(timer, 48i32 as uint16_t);
    let mut period: uint16_t =
        timerGetPeriodByPrescaler(timer, prescaler, 800000i32 as uint32_t);
    BIT_COMPARE_1 = (period as libc::c_int / 3i32 * 2i32) as uint16_t;
    BIT_COMPARE_0 = (period as libc::c_int / 3i32) as uint16_t;
    /* Time base configuration */
    TIM_TimeBaseStructInit(&mut TIM_TimeBaseStructure); // 800kHz
    TIM_TimeBaseStructure.TIM_Period = period as uint32_t;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0i32 as uint16_t;
    TIM_TimeBaseStructure.TIM_CounterMode = 0i32 as uint16_t;
    TIM_TimeBaseInit(timer, &mut TIM_TimeBaseStructure);
    /* PWM1 Mode configuration */
    TIM_OCStructInit(&mut TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = 0x60i32 as uint32_t;
    if (*timerHardware).output as libc::c_int &
           TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        TIM_OCInitStructure.TIM_OutputNState = 0x4i32 as uint16_t;
        TIM_OCInitStructure.TIM_OCNIdleState = 0i32 as uint16_t;
        TIM_OCInitStructure.TIM_OCNPolarity =
            if (*timerHardware).output as libc::c_int &
                   TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
                0x8i32 as uint16_t as libc::c_int
            } else { 0i32 as uint16_t as libc::c_int } as uint16_t
    } else {
        TIM_OCInitStructure.TIM_OutputState = 0x1i32 as uint16_t;
        TIM_OCInitStructure.TIM_OCIdleState = 0x100i32 as uint16_t;
        TIM_OCInitStructure.TIM_OCPolarity =
            if (*timerHardware).output as libc::c_int &
                   TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
                0x2i32 as uint16_t as libc::c_int
            } else { 0i32 as uint16_t as libc::c_int } as uint16_t
    }
    TIM_OCInitStructure.TIM_Pulse = 0i32 as uint32_t;
    timerOCInit(timer, (*timerHardware).channel, &mut TIM_OCInitStructure);
    timerOCPreloadConfig(timer, (*timerHardware).channel, 0x8i32 as uint16_t);
    TIM_CtrlPWMOutputs(timer, ENABLE);
    TIM_ARRPreloadConfig(timer, ENABLE);
    if (*timerHardware).output as libc::c_int &
           TIMER_OUTPUT_N_CHANNEL as libc::c_int != 0 {
        TIM_CCxNCmd(timer, (*timerHardware).channel as uint16_t,
                    0x4i32 as uint16_t);
    } else {
        TIM_CCxCmd(timer, (*timerHardware).channel as uint16_t,
                   0x1i32 as uint16_t);
    }
    TIM_Cmd(timer, ENABLE);
    dmaInit((*timerHardware).dmaIrqHandler as dmaIdentifier_e,
            OWNER_LED_STRIP, 0i32 as uint8_t);
    dmaSetHandler((*timerHardware).dmaIrqHandler as dmaIdentifier_e,
                  Some(WS2811_DMA_IRQHandler as
                           unsafe extern "C" fn(_:
                                                    *mut dmaChannelDescriptor_t)
                               -> ()),
                  ((1i32 <<
                        (4i32 as
                             libc::c_uint).wrapping_sub((7i32 as
                                                             libc::c_uint).wrapping_sub(0x500i32
                                                                                            as
                                                                                            uint32_t
                                                                                            >>
                                                                                            8i32))
                        |
                        2i32 &
                            0xfi32 >>
                                (7i32 as
                                     libc::c_uint).wrapping_sub(0x500i32 as
                                                                    uint32_t
                                                                    >> 8i32))
                       << 4i32 & 0xf0i32) as uint32_t, 0i32 as uint32_t);
    dmaRef = (*timerHardware).dmaRef;
    DMA_DeInit(dmaRef);
    /* configure DMA */
    DMA_Cmd(dmaRef, DISABLE); // load number of bytes to be transferred
    DMA_DeInit(dmaRef);
    DMA_StructInit(&mut DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr =
        timerCCR(timer, (*timerHardware).channel) as uint32_t;
    DMA_InitStructure.DMA_BufferSize = (24i32 * 32i32 + 42i32) as uint16_t;
    DMA_InitStructure.DMA_PeripheralInc = 0i32 as uint32_t;
    DMA_InitStructure.DMA_MemoryInc = 0x80i32 as uint32_t;
    DMA_InitStructure.DMA_MemoryBaseAddr =
        ledStripDMABuffer.as_mut_ptr() as uint32_t;
    DMA_InitStructure.DMA_DIR = 0x10i32 as uint32_t;
    DMA_InitStructure.DMA_PeripheralDataSize = 0x200i32 as uint32_t;
    DMA_InitStructure.DMA_MemoryDataSize = 0i32 as uint32_t;
    DMA_InitStructure.DMA_Priority = 0x2000i32 as uint32_t;
    DMA_InitStructure.DMA_M2M = 0i32 as uint32_t;
    DMA_InitStructure.DMA_Mode = 0i32 as uint32_t;
    DMA_Init(dmaRef, &mut DMA_InitStructure);
    TIM_DMACmd(timer, timerDmaSource((*timerHardware).channel), ENABLE);
    DMA_ITConfig(dmaRef, 0x2i32 as uint32_t, ENABLE);
    ws2811Initialised = 1i32 != 0;
}
#[no_mangle]
pub unsafe extern "C" fn ws2811LedStripDMAEnable() {
    if !ws2811Initialised { return }
    DMA_SetCurrDataCounter(dmaRef, (24i32 * 32i32 + 42i32) as uint16_t);
    TIM_SetCounter(timer, 0i32 as uint32_t);
    TIM_Cmd(timer, ENABLE);
    DMA_Cmd(dmaRef, ENABLE);
}
