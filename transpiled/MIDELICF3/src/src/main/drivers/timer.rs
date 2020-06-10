use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    #[no_mangle]
    fn TIM_TimeBaseInit(TIMx: *mut TIM_TypeDef,
                        TIM_TimeBaseInitStruct: *mut TIM_TimeBaseInitTypeDef);
    #[no_mangle]
    fn TIM_TimeBaseStructInit(TIM_TimeBaseInitStruct:
                                  *mut TIM_TimeBaseInitTypeDef);
    #[no_mangle]
    fn TIM_Cmd(TIMx: *mut TIM_TypeDef, NewState: FunctionalState);
    /* Output Compare management **************************************************/
    #[no_mangle]
    fn TIM_OC1Init(TIMx: *mut TIM_TypeDef,
                   TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_OC2Init(TIMx: *mut TIM_TypeDef,
                   TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_OC3Init(TIMx: *mut TIM_TypeDef,
                   TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_OC4Init(TIMx: *mut TIM_TypeDef,
                   TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_OCStructInit(TIM_OCInitStruct: *mut TIM_OCInitTypeDef);
    #[no_mangle]
    fn TIM_OC1PreloadConfig(TIMx: *mut TIM_TypeDef, TIM_OCPreload: uint16_t);
    #[no_mangle]
    fn TIM_OC2PreloadConfig(TIMx: *mut TIM_TypeDef, TIM_OCPreload: uint16_t);
    #[no_mangle]
    fn TIM_OC3PreloadConfig(TIMx: *mut TIM_TypeDef, TIM_OCPreload: uint16_t);
    #[no_mangle]
    fn TIM_OC4PreloadConfig(TIMx: *mut TIM_TypeDef, TIM_OCPreload: uint16_t);
    /* Input Capture management ***************************************************/
    #[no_mangle]
    fn TIM_ICInit(TIMx: *mut TIM_TypeDef,
                  TIM_ICInitStruct: *mut TIM_ICInitTypeDef);
    #[no_mangle]
    fn TIM_ICStructInit(TIM_ICInitStruct: *mut TIM_ICInitTypeDef);
    /* Interrupts, DMA and flags management ***************************************/
    #[no_mangle]
    fn TIM_ITConfig(TIMx: *mut TIM_TypeDef, TIM_IT: uint16_t,
                    NewState: FunctionalState);
    #[no_mangle]
    fn TIM_ClearFlag(TIMx: *mut TIM_TypeDef, TIM_FLAG: uint16_t);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    static timerHardware: [timerHardware_t; 0];
    #[no_mangle]
    static timerDefinitions: [timerDef_t; 0];
    #[no_mangle]
    fn timerClock(tim: *mut TIM_TypeDef) -> uint32_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
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
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
/* *
  * @brief TIM
  */
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
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_TimeBaseInitTypeDef {
    pub TIM_Prescaler: uint16_t,
    pub TIM_CounterMode: uint16_t,
    pub TIM_Period: uint32_t,
    pub TIM_ClockDivision: uint16_t,
    pub TIM_RepetitionCounter: uint16_t,
}
#[derive(Copy, Clone)]
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct TIM_ICInitTypeDef {
    pub TIM_Channel: uint16_t,
    pub TIM_ICPolarity: uint16_t,
    pub TIM_ICSelection: uint16_t,
    pub TIM_ICPrescaler: uint16_t,
    pub TIM_ICFilter: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_misc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the miscellaneous
  *          firmware library functions (add-on to CMSIS functions).
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
/* * @addtogroup MISC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  NVIC Init Structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct NVIC_InitTypeDef {
    pub NVIC_IRQChannel: uint8_t,
    pub NVIC_IRQChannelPreemptionPriority: uint8_t,
    pub NVIC_IRQChannelSubPriority: uint8_t,
    pub NVIC_IRQChannelCmd: FunctionalState,
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
// 16 bit on both 103 and 303, just register access must be 32bit sometimes (use timCCR_t)
pub type timCCR_t = uint32_t;
pub type timCCER_t = uint32_t;
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
pub struct timerCCHandlerRec_s {
    pub fn_0: Option<timerCCHandlerCallback>,
}
// use different types from capture and overflow - multiple overflow handlers are implemented as linked list
pub type timerCCHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerCCHandlerRec_s, _: uint16_t) -> ();
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerOvrHandlerRec_s {
    pub fn_0: Option<timerOvrHandlerCallback>,
    pub next: *mut timerOvrHandlerRec_s,
}
pub type timerOvrHandlerCallback
    =
    unsafe extern "C" fn(_: *mut timerOvrHandlerRec_s, _: uint16_t) -> ();
pub type timerCCHandlerRec_t = timerCCHandlerRec_s;
pub type timerOvrHandlerRec_t = timerOvrHandlerRec_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerDef_s {
    pub TIMx: *mut TIM_TypeDef,
    pub rcc: rccPeriphTag_t,
    pub inputIrq: uint8_t,
}
pub type timerDef_t = timerDef_s;
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
pub type C2RustUnnamed_2 = libc::c_uint;
pub const TIMER_OUTPUT_N_CHANNEL: C2RustUnnamed_2 = 2;
pub const TIMER_OUTPUT_INVERTED: C2RustUnnamed_2 = 1;
pub const TIMER_OUTPUT_NONE: C2RustUnnamed_2 = 0;
pub type channelType_t = libc::c_uint;
pub const TYPE_TIMER: channelType_t = 14;
pub const TYPE_SERIAL_RXTX: channelType_t = 13;
pub const TYPE_SERIAL_TX: channelType_t = 12;
pub const TYPE_SERIAL_RX: channelType_t = 11;
pub const TYPE_ADC: channelType_t = 10;
pub const TYPE_SOFTSERIAL_AUXTIMER: channelType_t = 9;
pub const TYPE_SOFTSERIAL_RXTX: channelType_t = 8;
pub const TYPE_SOFTSERIAL_TX: channelType_t = 7;
pub const TYPE_SOFTSERIAL_RX: channelType_t = 6;
pub const TYPE_PWMOUTPUT_SERVO: channelType_t = 5;
pub const TYPE_PWMOUTPUT_FAST: channelType_t = 4;
pub const TYPE_PWMOUTPUT_MOTOR: channelType_t = 3;
pub const TYPE_PPMINPUT: channelType_t = 2;
pub const TYPE_PWMINPUT: channelType_t = 1;
pub const TYPE_FREE: channelType_t = 0;
// TIMUP
// TIM_Channel_1..4
pub type timerConfig_t = timerConfig_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerConfig_s {
    pub edgeCallback: [*mut timerCCHandlerRec_t; 4],
    pub overflowCallback: [*mut timerOvrHandlerRec_t; 4],
    pub overflowCallbackActive: *mut timerOvrHandlerRec_t,
    pub forcedOverflowTimerValue: uint32_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerInfo_t {
    pub priority: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct timerChannelInfo_t {
    pub type_0: channelType_t,
}
// null-terminated linkded list of active overflow callbacks
// set BASEPRI_MAX, with global memory barrier, returns true
#[inline]
unsafe extern "C" fn __basepriSetMemRetVal(mut prio: uint8_t) -> uint8_t {
    __set_BASEPRI_MAX(prio as libc::c_int);
    return 1 as libc::c_int as uint8_t;
}
#[no_mangle]
pub static mut timerConfig: [timerConfig_t; 5] =
    [timerConfig_t{edgeCallback:
                       [0 as *const timerCCHandlerRec_t as
                            *mut timerCCHandlerRec_t; 4],
                   overflowCallback:
                       [0 as *const timerOvrHandlerRec_t as
                            *mut timerOvrHandlerRec_t; 4],
                   overflowCallbackActive:
                       0 as *const timerOvrHandlerRec_t as
                           *mut timerOvrHandlerRec_t,
                   forcedOverflowTimerValue: 0,}; 5];
#[no_mangle]
pub static mut timerChannelInfo: [timerChannelInfo_t; 6] =
    [timerChannelInfo_t{type_0: TYPE_FREE,}; 6];
#[no_mangle]
pub static mut timerInfo: [timerInfo_t; 5] = [timerInfo_t{priority: 0,}; 5];
// return index of timer in timer table. Lowest timer has index 0
unsafe extern "C" fn lookupTimerIndex(mut tim: *const TIM_TypeDef)
 -> uint8_t {
    // amount we can safely shift timer address to the right. gcc will throw error if some timers overlap
    // let gcc do the work, switch should be quite optimized
    match tim as libc::c_uint >> 10 as libc::c_int {
        1048651 => {
            return (((((1 as libc::c_int) << 1 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int)) -
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 1 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              -
                              ((((1 as libc::c_int) << 1 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 1 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 1 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
            // make sure final index is out of range
        }
        1048576 => {
            return (((((1 as libc::c_int) << 2 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int)) -
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 2 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              -
                              ((((1 as libc::c_int) << 2 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 2 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 2 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048577 => {
            return (((((1 as libc::c_int) << 3 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int)) -
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 3 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              -
                              ((((1 as libc::c_int) << 3 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 3 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 3 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048653 => {
            return (((((1 as libc::c_int) << 8 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int)) -
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 8 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              -
                              ((((1 as libc::c_int) << 8 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 8 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 8 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        1048656 => {
            return (((((1 as libc::c_int) << 15 as libc::c_int) -
                          1 as libc::c_int &
                          ((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int)) -
                         ((((1 as libc::c_int) << 15 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 1 as libc::c_int & 0x77777777 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 15 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 2 as libc::c_int & 0x33333333 as libc::c_int)
                         -
                         ((((1 as libc::c_int) << 15 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              >> 3 as libc::c_int & 0x11111111 as libc::c_int)
                         +
                         ((((1 as libc::c_int) << 15 as libc::c_int) -
                               1 as libc::c_int &
                               ((1 as libc::c_int) << 1 as libc::c_int |
                                    (1 as libc::c_int) << 2 as libc::c_int |
                                    (1 as libc::c_int) << 3 as libc::c_int |
                                    (1 as libc::c_int) << 8 as libc::c_int |
                                    (1 as libc::c_int) << 15 as libc::c_int))
                              -
                              ((((1 as libc::c_int) << 15 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   1 as libc::c_int &
                                   0x77777777 as libc::c_int) -
                              ((((1 as libc::c_int) << 15 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   2 as libc::c_int &
                                   0x33333333 as libc::c_int) -
                              ((((1 as libc::c_int) << 15 as libc::c_int) -
                                    1 as libc::c_int &
                                    ((1 as libc::c_int) << 1 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             2 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             3 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             8 as libc::c_int |
                                         (1 as libc::c_int) <<
                                             15 as libc::c_int)) >>
                                   3 as libc::c_int &
                                   0x11111111 as libc::c_int) >>
                              4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                        255 as libc::c_int) as uint8_t
        }
        _ => { return !(1 as libc::c_int) as uint8_t }
    };
}
// Initialized in run_static_initializers
#[no_mangle]
pub static mut usedTimers: [*mut TIM_TypeDef; 5] =
    [0 as *const TIM_TypeDef as *mut TIM_TypeDef; 5];
// Map timer index to timer number (Straight copy of usedTimers array)
#[no_mangle]
pub static mut timerNumbers: [int8_t; 5] =
    [1 as libc::c_int as int8_t, 2 as libc::c_int as int8_t,
     3 as libc::c_int as int8_t, 8 as libc::c_int as int8_t,
     15 as libc::c_int as int8_t];
#[no_mangle]
pub unsafe extern "C" fn timerGetTIMNumber(mut tim: *const TIM_TypeDef)
 -> int8_t {
    let mut index: uint8_t = lookupTimerIndex(tim);
    if (index as libc::c_int) <
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 15 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return timerNumbers[index as usize]
    } else { return 0 as libc::c_int as int8_t };
}
#[inline]
unsafe extern "C" fn lookupChannelIndex(channel: uint16_t) -> uint8_t {
    return (channel as libc::c_int >> 2 as libc::c_int) as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerLookupChannelIndex(channel: uint16_t)
 -> uint8_t {
    return lookupChannelIndex(channel);
}
// TODO - just for migration
#[no_mangle]
pub unsafe extern "C" fn timerRCC(mut tim: *mut TIM_TypeDef)
 -> rccPeriphTag_t {
    let mut i: libc::c_int = 0 as libc::c_int; // AKA TIMx_ARR
    while i < 10 as libc::c_int {
        if (*timerDefinitions.as_ptr().offset(i as isize)).TIMx == tim {
            return (*timerDefinitions.as_ptr().offset(i as isize)).rcc
        }
        i += 1
    }
    return 0 as libc::c_int as rccPeriphTag_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerInputIrq(mut tim: *mut TIM_TypeDef) -> uint8_t {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < 10 as libc::c_int {
        if (*timerDefinitions.as_ptr().offset(i as isize)).TIMx == tim {
            return (*timerDefinitions.as_ptr().offset(i as isize)).inputIrq
        }
        i += 1
    }
    return 0 as libc::c_int as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerNVICConfigure(mut irq: uint8_t) {
    let mut NVIC_InitStructure: NVIC_InitTypeDef =
        NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                         NVIC_IRQChannelPreemptionPriority: 0,
                         NVIC_IRQChannelSubPriority: 0,
                         NVIC_IRQChannelCmd: DISABLE,};
    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
        ((((1 as libc::c_int) <<
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
               |
               1 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int) >>
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
        ((((1 as libc::c_int) <<
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
               |
               1 as libc::c_int &
                   0xf as libc::c_int >>
                       (7 as libc::c_int as
                            libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                           uint32_t >>
                                                           8 as libc::c_int))
              << 4 as libc::c_int & 0xf0 as libc::c_int &
              0xf as libc::c_int >>
                  (7 as libc::c_int as
                       libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                      uint32_t >>
                                                      8 as libc::c_int)) >>
             4 as libc::c_int) as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&mut NVIC_InitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn configTimeBase(mut tim: *mut TIM_TypeDef,
                                        mut period: uint16_t,
                                        mut hz: uint32_t) {
    let mut TIM_TimeBaseStructure: TIM_TimeBaseInitTypeDef =
        TIM_TimeBaseInitTypeDef{TIM_Prescaler: 0,
                                TIM_CounterMode: 0,
                                TIM_Period: 0,
                                TIM_ClockDivision: 0,
                                TIM_RepetitionCounter: 0,};
    TIM_TimeBaseStructInit(&mut TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period =
        (period as libc::c_int - 1 as libc::c_int & 0xffff as libc::c_int) as
            uint32_t;
    // "The counter clock frequency (CK_CNT) is equal to f CK_PSC / (PSC[15:0] + 1)." - STM32F10x Reference Manual 14.4.11
    // Thus for 1Mhz: 72000000 / 1000000 = 72, 72 - 1 = 71 = TIM_Prescaler
    TIM_TimeBaseStructure.TIM_Prescaler =
        timerClock(tim).wrapping_div(hz).wrapping_sub(1 as libc::c_int as
                                                          libc::c_uint) as
            uint16_t;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0 as libc::c_int as uint16_t;
    TIM_TimeBaseStructure.TIM_CounterMode = 0 as libc::c_int as uint16_t;
    TIM_TimeBaseInit(tim, &mut TIM_TimeBaseStructure);
}
// old interface for PWM inputs. It should be replaced
#[no_mangle]
pub unsafe extern "C" fn timerConfigure(mut timerHardwarePtr:
                                            *const timerHardware_t,
                                        mut period: uint16_t,
                                        mut hz: uint32_t) {
    configTimeBase((*timerHardwarePtr).tim, period, hz);
    TIM_Cmd((*timerHardwarePtr).tim, ENABLE);
    let mut irq: uint8_t = timerInputIrq((*timerHardwarePtr).tim);
    timerNVICConfigure(irq);
    // HACK - enable second IRQ on timers that need it
    match irq as libc::c_int {
        27 => {
            timerNVICConfigure(TIM1_UP_TIM16_IRQn as libc::c_int as uint8_t);
        }
        _ => { }
    };
}
// allocate and configure timer channel. Timer priority is set to highest priority of its channels
#[no_mangle]
pub unsafe extern "C" fn timerChInit(mut timHw: *const timerHardware_t,
                                     mut type_0: channelType_t,
                                     mut irqPriority: libc::c_int,
                                     mut irq: uint8_t) {
    let mut channel: libc::c_uint =
        timHw.wrapping_offset_from(timerHardware.as_ptr()) as libc::c_long as
            libc::c_uint;
    if channel >= 6 as libc::c_int as libc::c_uint { return }
    timerChannelInfo[channel as usize].type_0 = type_0;
    let mut timer: libc::c_uint =
        lookupTimerIndex((*timHw).tim) as libc::c_uint;
    if timer >=
           ((((1 as libc::c_int) << 1 as libc::c_int |
                  (1 as libc::c_int) << 2 as libc::c_int |
                  (1 as libc::c_int) << 3 as libc::c_int |
                  (1 as libc::c_int) << 8 as libc::c_int |
                  (1 as libc::c_int) << 15 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 15 as libc::c_int) >>
                      1 as libc::c_int & 0x77777777 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 15 as libc::c_int) >>
                      2 as libc::c_int & 0x33333333 as libc::c_int) -
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 15 as libc::c_int) >>
                      3 as libc::c_int & 0x11111111 as libc::c_int) +
                 (((1 as libc::c_int) << 1 as libc::c_int |
                       (1 as libc::c_int) << 2 as libc::c_int |
                       (1 as libc::c_int) << 3 as libc::c_int |
                       (1 as libc::c_int) << 8 as libc::c_int |
                       (1 as libc::c_int) << 15 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 15 as libc::c_int) >>
                           1 as libc::c_int & 0x77777777 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 15 as libc::c_int) >>
                           2 as libc::c_int & 0x33333333 as libc::c_int) -
                      (((1 as libc::c_int) << 1 as libc::c_int |
                            (1 as libc::c_int) << 2 as libc::c_int |
                            (1 as libc::c_int) << 3 as libc::c_int |
                            (1 as libc::c_int) << 8 as libc::c_int |
                            (1 as libc::c_int) << 15 as libc::c_int) >>
                           3 as libc::c_int & 0x11111111 as libc::c_int) >>
                      4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                255 as libc::c_int) as libc::c_uint {
        return
    }
    if irqPriority < timerInfo[timer as usize].priority as libc::c_int {
        // it would be better to set priority in the end, but current startup sequence is not ready
        configTimeBase(usedTimers[timer as usize],
                       0 as libc::c_int as uint16_t,
                       1 as libc::c_int as uint32_t);
        TIM_Cmd(usedTimers[timer as usize], ENABLE);
        let mut NVIC_InitStructure: NVIC_InitTypeDef =
            NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                             NVIC_IRQChannelPreemptionPriority: 0,
                             NVIC_IRQChannelSubPriority: 0,
                             NVIC_IRQChannelCmd: DISABLE,};
        NVIC_InitStructure.NVIC_IRQChannel = irq;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
            (irqPriority >>
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
            ((irqPriority &
                  0xf as libc::c_int >>
                      (7 as libc::c_int as
                           libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                          uint32_t >>
                                                          8 as libc::c_int))
                 >> 4 as libc::c_int) as uint8_t;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&mut NVIC_InitStructure);
        timerInfo[timer as usize].priority = irqPriority as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCHandlerInit(mut self_0:
                                                  *mut timerCCHandlerRec_t,
                                              mut fn_0:
                                                  Option<timerCCHandlerCallback>) {
    (*self_0).fn_0 = fn_0;
}
#[no_mangle]
pub unsafe extern "C" fn timerChOvrHandlerInit(mut self_0:
                                                   *mut timerOvrHandlerRec_t,
                                               mut fn_0:
                                                   Option<timerOvrHandlerCallback>) {
    (*self_0).fn_0 = fn_0;
    (*self_0).next = 0 as *mut timerOvrHandlerRec_s;
}
// update overflow callback list
// some synchronization mechanism is neccesary to avoid disturbing other channels (BASEPRI used now)
unsafe extern "C" fn timerChConfig_UpdateOverflow(mut cfg: *mut timerConfig_t,
                                                  mut tim: *mut TIM_TypeDef) {
    let mut chain: *mut *mut timerOvrHandlerRec_t =
        &mut (*cfg).overflowCallbackActive;
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((1 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
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
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        let mut i: libc::c_int = 0 as libc::c_int;
        while i < 4 as libc::c_int {
            if !(*cfg).overflowCallback[i as usize].is_null() {
                *chain = (*cfg).overflowCallback[i as usize];
                chain =
                    &mut (**(*cfg).overflowCallback.as_mut_ptr().offset(i as
                                                                            isize)).next
            }
            i += 1
        }
        *chain = 0 as *mut timerOvrHandlerRec_t;
        __ToDo = 0 as libc::c_int as uint8_t
    }
    // enable or disable IRQ
    TIM_ITConfig(tim, 0x1 as libc::c_int as uint16_t,
                 if !(*cfg).overflowCallbackActive.is_null() {
                     ENABLE as libc::c_int
                 } else { DISABLE as libc::c_int } as FunctionalState);
}
// config edge and overflow callback for channel. Try to avoid overflowCallback, it is a bit expensive
#[no_mangle]
pub unsafe extern "C" fn timerChConfigCallbacks(mut timHw:
                                                    *const timerHardware_t,
                                                mut edgeCallback:
                                                    *mut timerCCHandlerRec_t,
                                                mut overflowCallback:
                                                    *mut timerOvrHandlerRec_t) {
    let mut timerIndex: uint8_t = lookupTimerIndex((*timHw).tim);
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 15 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    }
    let mut channelIndex: uint8_t =
        lookupChannelIndex((*timHw).channel as uint16_t);
    if edgeCallback.is_null() {
        // disable irq before changing callback to NULL
        TIM_ITConfig((*timHw).tim,
                     ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                          (*timHw).channel as libc::c_int / 4 as libc::c_int)
                         as uint16_t, DISABLE);
    }
    // setup callback info
    timerConfig[timerIndex as usize].edgeCallback[channelIndex as usize] =
        edgeCallback;
    timerConfig[timerIndex as usize].overflowCallback[channelIndex as usize] =
        overflowCallback;
    // enable channel IRQ
    if !edgeCallback.is_null() {
        TIM_ITConfig((*timHw).tim,
                     ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                          (*timHw).channel as libc::c_int / 4 as libc::c_int)
                         as uint16_t, ENABLE);
    }
    timerChConfig_UpdateOverflow(&mut *timerConfig.as_mut_ptr().offset(timerIndex
                                                                           as
                                                                           isize),
                                 (*timHw).tim);
}
// configure callbacks for pair of channels (1+2 or 3+4).
// Hi(2,4) and Lo(1,3) callbacks are specified, it is not important which timHw channel is used.
// This is intended for dual capture mode (each channel handles one transition)
#[no_mangle]
pub unsafe extern "C" fn timerChConfigCallbacksDual(mut timHw:
                                                        *const timerHardware_t,
                                                    mut edgeCallbackLo:
                                                        *mut timerCCHandlerRec_t,
                                                    mut edgeCallbackHi:
                                                        *mut timerCCHandlerRec_t,
                                                    mut overflowCallback:
                                                        *mut timerOvrHandlerRec_t) {
    let mut timerIndex: uint8_t =
        lookupTimerIndex((*timHw).tim); // lower channel
    if timerIndex as libc::c_int >=
           (((1 as libc::c_int) << 1 as libc::c_int |
                 (1 as libc::c_int) << 2 as libc::c_int |
                 (1 as libc::c_int) << 3 as libc::c_int |
                 (1 as libc::c_int) << 8 as libc::c_int |
                 (1 as libc::c_int) << 15 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     1 as libc::c_int & 0x77777777 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     2 as libc::c_int & 0x33333333 as libc::c_int) -
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) >>
                     3 as libc::c_int & 0x11111111 as libc::c_int) +
                (((1 as libc::c_int) << 1 as libc::c_int |
                      (1 as libc::c_int) << 2 as libc::c_int |
                      (1 as libc::c_int) << 3 as libc::c_int |
                      (1 as libc::c_int) << 8 as libc::c_int |
                      (1 as libc::c_int) << 15 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          1 as libc::c_int & 0x77777777 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          2 as libc::c_int & 0x33333333 as libc::c_int) -
                     (((1 as libc::c_int) << 1 as libc::c_int |
                           (1 as libc::c_int) << 2 as libc::c_int |
                           (1 as libc::c_int) << 3 as libc::c_int |
                           (1 as libc::c_int) << 8 as libc::c_int |
                           (1 as libc::c_int) << 15 as libc::c_int) >>
                          3 as libc::c_int & 0x11111111 as libc::c_int) >>
                     4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
               255 as libc::c_int {
        return
    } // upper channel
    let mut chLo: uint16_t =
        ((*timHw).channel as libc::c_int &
             !(0x4 as libc::c_int as uint16_t as libc::c_int)) as
            uint16_t; // get index of lower channel
    let mut chHi: uint16_t =
        ((*timHw).channel as libc::c_int |
             0x4 as libc::c_int as uint16_t as libc::c_int) as uint16_t;
    let mut channelIndex: uint8_t = lookupChannelIndex(chLo);
    if edgeCallbackLo.is_null() {
        // disable irq before changing setting callback to NULL
        TIM_ITConfig((*timHw).tim,
                     ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                          chLo as libc::c_int / 4 as libc::c_int) as uint16_t,
                     DISABLE);
    }
    if edgeCallbackHi.is_null() {
        // disable irq before changing setting callback to NULL
        TIM_ITConfig((*timHw).tim,
                     ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                          chHi as libc::c_int / 4 as libc::c_int) as uint16_t,
                     DISABLE);
    }
    // setup callback info
    timerConfig[timerIndex as usize].edgeCallback[channelIndex as usize] =
        edgeCallbackLo;
    timerConfig[timerIndex as
                    usize].edgeCallback[(channelIndex as libc::c_int +
                                             1 as libc::c_int) as usize] =
        edgeCallbackHi;
    timerConfig[timerIndex as usize].overflowCallback[channelIndex as usize] =
        overflowCallback;
    timerConfig[timerIndex as
                    usize].overflowCallback[(channelIndex as libc::c_int +
                                                 1 as libc::c_int) as usize] =
        0 as *mut timerOvrHandlerRec_t;
    // enable channel IRQs
    if !edgeCallbackLo.is_null() {
        TIM_ClearFlag((*timHw).tim,
                      ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                           chLo as libc::c_int / 4 as libc::c_int) as
                          uint16_t);
        TIM_ITConfig((*timHw).tim,
                     ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                          chLo as libc::c_int / 4 as libc::c_int) as uint16_t,
                     ENABLE);
    }
    if !edgeCallbackHi.is_null() {
        TIM_ClearFlag((*timHw).tim,
                      ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                           chHi as libc::c_int / 4 as libc::c_int) as
                          uint16_t);
        TIM_ITConfig((*timHw).tim,
                     ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                          chHi as libc::c_int / 4 as libc::c_int) as uint16_t,
                     ENABLE);
    }
    timerChConfig_UpdateOverflow(&mut *timerConfig.as_mut_ptr().offset(timerIndex
                                                                           as
                                                                           isize),
                                 (*timHw).tim);
}
// enable/disable IRQ for low channel in dual configuration
#[no_mangle]
pub unsafe extern "C" fn timerChITConfigDualLo(mut timHw:
                                                   *const timerHardware_t,
                                               mut newState:
                                                   FunctionalState) {
    TIM_ITConfig((*timHw).tim,
                 ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                      ((*timHw).channel as libc::c_int &
                           !(0x4 as libc::c_int as uint16_t as libc::c_int)) /
                          4 as libc::c_int) as uint16_t, newState);
}
// enable or disable IRQ
#[no_mangle]
pub unsafe extern "C" fn timerChITConfig(mut timHw: *const timerHardware_t,
                                         mut newState: FunctionalState) {
    TIM_ITConfig((*timHw).tim,
                 ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                      (*timHw).channel as libc::c_int / 4 as libc::c_int) as
                     uint16_t, newState);
}
// clear Compare/Capture flag for channel
#[no_mangle]
pub unsafe extern "C" fn timerChClearCCFlag(mut timHw:
                                                *const timerHardware_t) {
    TIM_ClearFlag((*timHw).tim,
                  ((0x2 as libc::c_int as uint16_t as libc::c_int) <<
                       (*timHw).channel as libc::c_int / 4 as libc::c_int) as
                      uint16_t);
}
// configure timer channel GPIO mode
#[no_mangle]
pub unsafe extern "C" fn timerChConfigGPIO(mut timHw: *const timerHardware_t,
                                           mut mode: ioConfig_t) {
    IOInit(IOGetByTag((*timHw).tag), OWNER_TIMER,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO(IOGetByTag((*timHw).tag), mode);
}
// calculate input filter constant
// TODO - we should probably setup DTS to higher value to allow reasonable input filtering
//   - notice that prescaler[0] does use DTS for sampling - the sequence won't be monotonous anymore
unsafe extern "C" fn getFilter(mut ticks: libc::c_uint) -> libc::c_uint {
    static mut ftab: [libc::c_uint; 16] =
        [(1 as libc::c_int * 1 as libc::c_int) as libc::c_uint,
         (1 as libc::c_int * 2 as libc::c_int) as libc::c_uint,
         (1 as libc::c_int * 4 as libc::c_int) as libc::c_uint,
         (1 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (2 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (2 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (4 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (4 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (8 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (8 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (16 as libc::c_int * 5 as libc::c_int) as libc::c_uint,
         (16 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (16 as libc::c_int * 8 as libc::c_int) as libc::c_uint,
         (32 as libc::c_int * 5 as libc::c_int) as libc::c_uint,
         (32 as libc::c_int * 6 as libc::c_int) as libc::c_uint,
         (32 as libc::c_int * 8 as libc::c_int) as libc::c_uint];
    let mut i: libc::c_uint = 1 as libc::c_int as libc::c_uint;
    while (i as libc::c_ulong) <
              (::core::mem::size_of::<[libc::c_uint; 16]>() as
                   libc::c_ulong).wrapping_div(::core::mem::size_of::<libc::c_uint>()
                                                   as libc::c_ulong) {
        if ftab[i as usize] > ticks {
            return i.wrapping_sub(1 as libc::c_int as libc::c_uint)
        }
        i = i.wrapping_add(1)
    }
    return 0xf as libc::c_int as libc::c_uint;
}
// This interface should be replaced.
// Configure input captupre
#[no_mangle]
pub unsafe extern "C" fn timerChConfigIC(mut timHw: *const timerHardware_t,
                                         mut polarityRising: bool,
                                         mut inputFilterTicks: libc::c_uint) {
    let mut TIM_ICInitStructure: TIM_ICInitTypeDef =
        TIM_ICInitTypeDef{TIM_Channel: 0,
                          TIM_ICPolarity: 0,
                          TIM_ICSelection: 0,
                          TIM_ICPrescaler: 0,
                          TIM_ICFilter: 0,};
    TIM_ICStructInit(&mut TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = (*timHw).channel as uint16_t;
    TIM_ICInitStructure.TIM_ICPolarity =
        if polarityRising as libc::c_int != 0 {
            0 as libc::c_int as uint16_t as libc::c_int
        } else { 0x2 as libc::c_int as uint16_t as libc::c_int } as uint16_t;
    TIM_ICInitStructure.TIM_ICSelection = 0x1 as libc::c_int as uint16_t;
    TIM_ICInitStructure.TIM_ICPrescaler = 0 as libc::c_int as uint16_t;
    TIM_ICInitStructure.TIM_ICFilter =
        getFilter(inputFilterTicks) as uint16_t;
    TIM_ICInit((*timHw).tim, &mut TIM_ICInitStructure);
}
// configure dual channel input channel for capture
// polarity is for Low channel (capture order is always Lo - Hi)
#[no_mangle]
pub unsafe extern "C" fn timerChConfigICDual(mut timHw:
                                                 *const timerHardware_t,
                                             mut polarityRising: bool,
                                             mut inputFilterTicks:
                                                 libc::c_uint) {
    let mut TIM_ICInitStructure: TIM_ICInitTypeDef =
        TIM_ICInitTypeDef{TIM_Channel: 0,
                          TIM_ICPolarity: 0,
                          TIM_ICSelection: 0,
                          TIM_ICPrescaler: 0,
                          TIM_ICFilter: 0,};
    let mut directRising: bool =
        if (*timHw).channel as libc::c_int &
               0x4 as libc::c_int as uint16_t as libc::c_int != 0 {
            !polarityRising as libc::c_int
        } else { polarityRising as libc::c_int } != 0;
    // configure direct channel
    TIM_ICStructInit(&mut TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = (*timHw).channel as uint16_t;
    TIM_ICInitStructure.TIM_ICPolarity =
        if directRising as libc::c_int != 0 {
            0 as libc::c_int as uint16_t as libc::c_int
        } else { 0x2 as libc::c_int as uint16_t as libc::c_int } as uint16_t;
    TIM_ICInitStructure.TIM_ICSelection = 0x1 as libc::c_int as uint16_t;
    TIM_ICInitStructure.TIM_ICPrescaler = 0 as libc::c_int as uint16_t;
    TIM_ICInitStructure.TIM_ICFilter =
        getFilter(inputFilterTicks) as uint16_t;
    TIM_ICInit((*timHw).tim, &mut TIM_ICInitStructure);
    // configure indirect channel
    TIM_ICInitStructure.TIM_Channel =
        ((*timHw).channel as libc::c_int ^
             0x4 as libc::c_int as uint16_t as libc::c_int) as
            uint16_t; // get opposite channel no
    TIM_ICInitStructure.TIM_ICPolarity =
        if directRising as libc::c_int != 0 {
            0x2 as libc::c_int as uint16_t as libc::c_int
        } else { 0 as libc::c_int as uint16_t as libc::c_int } as uint16_t;
    TIM_ICInitStructure.TIM_ICSelection = 0x2 as libc::c_int as uint16_t;
    TIM_ICInit((*timHw).tim, &mut TIM_ICInitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn timerChICPolarity(mut timHw: *const timerHardware_t,
                                           mut polarityRising: bool) {
    let mut tmpccer: timCCER_t = (*(*timHw).tim).CCER;
    tmpccer &=
        !((0x2 as libc::c_int as uint32_t) <<
              (*timHw).channel as libc::c_int);
    tmpccer |=
        if polarityRising as libc::c_int != 0 {
            ((0 as libc::c_int as uint16_t as libc::c_int)) <<
                (*timHw).channel as libc::c_int
        } else {
            ((0x2 as libc::c_int as uint16_t as libc::c_int)) <<
                (*timHw).channel as libc::c_int
        } as libc::c_uint;
    ::core::ptr::write_volatile(&mut (*(*timHw).tim).CCER as *mut uint32_t,
                                tmpccer);
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCRHi(mut timHw: *const timerHardware_t)
 -> *mut timCCR_t {
    return (&mut (*(*timHw).tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset(((*timHw).channel as libc::c_int |
                                               0x4 as libc::c_int as uint16_t
                                                   as libc::c_int) as isize)
               as *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCRLo(mut timHw: *const timerHardware_t)
 -> *mut timCCR_t {
    return (&mut (*(*timHw).tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset(((*timHw).channel as libc::c_int &
                                               !(0x4 as libc::c_int as
                                                     uint16_t as libc::c_int))
                                              as isize) as *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerChCCR(mut timHw: *const timerHardware_t)
 -> *mut timCCR_t {
    return (&mut (*(*timHw).tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset((*timHw).channel as libc::c_int as
                                              isize) as *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerChConfigOC(mut timHw: *const timerHardware_t,
                                         mut outEnable: bool,
                                         mut stateHigh: bool) {
    let mut TIM_OCInitStructure: TIM_OCInitTypeDef =
        TIM_OCInitTypeDef{TIM_OCMode: 0,
                          TIM_OutputState: 0,
                          TIM_OutputNState: 0,
                          TIM_Pulse: 0,
                          TIM_OCPolarity: 0,
                          TIM_OCNPolarity: 0,
                          TIM_OCIdleState: 0,
                          TIM_OCNIdleState: 0,};
    TIM_OCStructInit(&mut TIM_OCInitStructure);
    if outEnable {
        TIM_OCInitStructure.TIM_OCMode = 0x20 as libc::c_int as uint32_t;
        TIM_OCInitStructure.TIM_OutputState = 0x1 as libc::c_int as uint16_t;
        if (*timHw).output as libc::c_int &
               TIMER_OUTPUT_INVERTED as libc::c_int != 0 {
            stateHigh = !stateHigh
        }
        TIM_OCInitStructure.TIM_OCPolarity =
            if stateHigh as libc::c_int != 0 {
                0 as libc::c_int as uint16_t as libc::c_int
            } else { 0x2 as libc::c_int as uint16_t as libc::c_int } as
                uint16_t
    } else { TIM_OCInitStructure.TIM_OCMode = 0 as libc::c_int as uint32_t }
    match (*timHw).channel as libc::c_int {
        0 => {
            TIM_OC1Init((*timHw).tim, &mut TIM_OCInitStructure);
            TIM_OC1PreloadConfig((*timHw).tim, 0 as libc::c_int as uint16_t);
        }
        4 => {
            TIM_OC2Init((*timHw).tim, &mut TIM_OCInitStructure);
            TIM_OC2PreloadConfig((*timHw).tim, 0 as libc::c_int as uint16_t);
        }
        8 => {
            TIM_OC3Init((*timHw).tim, &mut TIM_OCInitStructure);
            TIM_OC3PreloadConfig((*timHw).tim, 0 as libc::c_int as uint16_t);
        }
        12 => {
            TIM_OC4Init((*timHw).tim, &mut TIM_OCInitStructure);
            TIM_OC4PreloadConfig((*timHw).tim, 0 as libc::c_int as uint16_t);
        }
        _ => { }
    };
}
unsafe extern "C" fn timCCxHandler(mut tim: *mut TIM_TypeDef,
                                   mut timerConfig_0: *mut timerConfig_t) {
    let mut capture: uint16_t = 0;
    let mut tim_status: libc::c_uint = 0;
    tim_status = (*tim).SR & (*tim).DIER;
    while tim_status != 0 {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        let mut bit: libc::c_uint =
            tim_status.leading_zeros() as i32 as libc::c_uint;
        let mut mask: libc::c_uint = !(0x80000000 as libc::c_uint >> bit);
        ::core::ptr::write_volatile(&mut (*tim).SR as *mut uint32_t, mask);
        tim_status &= mask;
        match bit {
            31 => {
                if (*timerConfig_0).forcedOverflowTimerValue !=
                       0 as libc::c_int as libc::c_uint {
                    capture =
                        (*timerConfig_0).forcedOverflowTimerValue.wrapping_sub(1
                                                                                   as
                                                                                   libc::c_int
                                                                                   as
                                                                                   libc::c_uint)
                            as uint16_t;
                    (*timerConfig_0).forcedOverflowTimerValue =
                        0 as libc::c_int as uint32_t
                } else { capture = (*tim).ARR as uint16_t }
                let mut cb: *mut timerOvrHandlerRec_t =
                    (*timerConfig_0).overflowCallbackActive;
                while !cb.is_null() {
                    (*cb).fn_0.expect("non-null function pointer")(cb,
                                                                   capture);
                    cb = (*cb).next
                }
            }
            30 => {
                (*(*timerConfig_0).edgeCallback[0 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[0
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR1
                                                                                                         as
                                                                                                         uint16_t);
            }
            29 => {
                (*(*timerConfig_0).edgeCallback[1 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[1
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR2
                                                                                                         as
                                                                                                         uint16_t);
            }
            28 => {
                (*(*timerConfig_0).edgeCallback[2 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[2
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR3
                                                                                                         as
                                                                                                         uint16_t);
            }
            27 => {
                (*(*timerConfig_0).edgeCallback[3 as libc::c_int as
                                                    usize]).fn_0.expect("non-null function pointer")((*timerConfig_0).edgeCallback[3
                                                                                                                                       as
                                                                                                                                       libc::c_int
                                                                                                                                       as
                                                                                                                                       usize],
                                                                                                     (*tim).CCR4
                                                                                                         as
                                                                                                         uint16_t);
            }
            _ => { }
        }
    };
}
// handler for shared interrupts when both timers need to check status bits
#[no_mangle]
pub unsafe extern "C" fn TIM1_CC_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x2c00
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                1 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM1_UP_TIM16_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x2c00
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                1 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     1 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          1 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
// timer16 is not used
#[no_mangle]
pub unsafe extern "C" fn TIM2_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0 as libc::c_int as
                                                  libc::c_uint) as
                      *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                2 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     2 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          2 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          2 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          2 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM3_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x400 as libc::c_int as
                                                  libc::c_uint) as
                      *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                3 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     3 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          3 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          3 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          3 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM8_CC_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x3400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                8 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
// f10x_hd, f30x
#[no_mangle]
pub unsafe extern "C" fn TIM8_UP_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x3400
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                8 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     8 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          8 as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn TIM1_BRK_TIM15_IRQHandler() {
    timCCxHandler((0x40000000 as libc::c_int as
                       uint32_t).wrapping_add(0x10000 as libc::c_int as
                                                  libc::c_uint).wrapping_add(0x4000
                                                                                 as
                                                                                 libc::c_int
                                                                                 as
                                                                                 libc::c_uint)
                      as *mut TIM_TypeDef,
                  &mut *timerConfig.as_mut_ptr().offset((((((1 as libc::c_int)
                                                                <<
                                                                15 as
                                                                    libc::c_int)
                                                               -
                                                               1 as
                                                                   libc::c_int
                                                               &
                                                               ((1 as
                                                                     libc::c_int)
                                                                    <<
                                                                    1 as
                                                                        libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        2 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        3 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        8 as
                                                                            libc::c_int
                                                                    |
                                                                    (1 as
                                                                         libc::c_int)
                                                                        <<
                                                                        15 as
                                                                            libc::c_int))
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     15 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   1 as
                                                                       libc::c_int
                                                                   &
                                                                   0x77777777
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     15 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   2 as
                                                                       libc::c_int
                                                                   &
                                                                   0x33333333
                                                                       as
                                                                       libc::c_int)
                                                              -
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     15 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   >>
                                                                   3 as
                                                                       libc::c_int
                                                                   &
                                                                   0x11111111
                                                                       as
                                                                       libc::c_int)
                                                              +
                                                              ((((1 as
                                                                      libc::c_int)
                                                                     <<
                                                                     15 as
                                                                         libc::c_int)
                                                                    -
                                                                    1 as
                                                                        libc::c_int
                                                                    &
                                                                    ((1 as
                                                                          libc::c_int)
                                                                         <<
                                                                         1 as
                                                                             libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             2
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             3
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             8
                                                                                 as
                                                                                 libc::c_int
                                                                         |
                                                                         (1 as
                                                                              libc::c_int)
                                                                             <<
                                                                             15
                                                                                 as
                                                                                 libc::c_int))
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        1 as
                                                                            libc::c_int
                                                                        &
                                                                        0x77777777
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        2 as
                                                                            libc::c_int
                                                                        &
                                                                        0x33333333
                                                                            as
                                                                            libc::c_int)
                                                                   -
                                                                   ((((1 as
                                                                           libc::c_int)
                                                                          <<
                                                                          15
                                                                              as
                                                                              libc::c_int)
                                                                         -
                                                                         1 as
                                                                             libc::c_int
                                                                         &
                                                                         ((1
                                                                               as
                                                                               libc::c_int)
                                                                              <<
                                                                              1
                                                                                  as
                                                                                  libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  2
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  3
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  8
                                                                                      as
                                                                                      libc::c_int
                                                                              |
                                                                              (1
                                                                                   as
                                                                                   libc::c_int)
                                                                                  <<
                                                                                  15
                                                                                      as
                                                                                      libc::c_int))
                                                                        >>
                                                                        3 as
                                                                            libc::c_int
                                                                        &
                                                                        0x11111111
                                                                            as
                                                                            libc::c_int)
                                                                   >>
                                                                   4 as
                                                                       libc::c_int)
                                                              &
                                                              0xf0f0f0f as
                                                                  libc::c_int)
                                                             %
                                                             255 as
                                                                 libc::c_int)
                                                            as isize));
}
#[no_mangle]
pub unsafe extern "C" fn timerInit() {
    memset(timerConfig.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[timerConfig_t; 5]>() as libc::c_ulong);
    /* enable the timer peripherals */
    let mut i: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i < 6 as libc::c_int as libc::c_uint {
        RCC_ClockCmd(timerRCC((*timerHardware.as_ptr().offset(i as
                                                                  isize)).tim),
                     ENABLE);
        i = i.wrapping_add(1)
    }
    let mut timerIndex: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while timerIndex < 6 as libc::c_int as libc::c_uint {
        let mut timerHardwarePtr: *const timerHardware_t =
            &*timerHardware.as_ptr().offset(timerIndex as isize) as
                *const timerHardware_t;
        if !((*timerHardwarePtr).usageFlags as libc::c_uint ==
                 TIM_USE_NONE as libc::c_int as libc::c_uint) {
            // XXX IOConfigGPIOAF in timerInit should eventually go away.
            IOConfigGPIOAF(IOGetByTag((*timerHardwarePtr).tag),
                           (GPIO_Mode_AF as libc::c_int |
                                (0 as libc::c_int) << 2 as libc::c_int |
                                (GPIO_OType_PP as libc::c_int) <<
                                    4 as libc::c_int |
                                (GPIO_PuPd_NOPULL as libc::c_int) <<
                                    5 as libc::c_int) as ioConfig_t,
                           (*timerHardwarePtr).alternateFunction);
        }
        timerIndex = timerIndex.wrapping_add(1)
    }
    // initialize timer channel structures
    let mut i_0: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i_0 < 6 as libc::c_int as libc::c_uint {
        timerChannelInfo[i_0 as usize].type_0 = TYPE_FREE;
        i_0 = i_0.wrapping_add(1)
    }
    let mut i_1: libc::c_uint = 0 as libc::c_int as libc::c_uint;
    while i_1 <
              ((((1 as libc::c_int) << 1 as libc::c_int |
                     (1 as libc::c_int) << 2 as libc::c_int |
                     (1 as libc::c_int) << 3 as libc::c_int |
                     (1 as libc::c_int) << 8 as libc::c_int |
                     (1 as libc::c_int) << 15 as libc::c_int) -
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 15 as libc::c_int) >>
                         1 as libc::c_int & 0x77777777 as libc::c_int) -
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 15 as libc::c_int) >>
                         2 as libc::c_int & 0x33333333 as libc::c_int) -
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 15 as libc::c_int) >>
                         3 as libc::c_int & 0x11111111 as libc::c_int) +
                    (((1 as libc::c_int) << 1 as libc::c_int |
                          (1 as libc::c_int) << 2 as libc::c_int |
                          (1 as libc::c_int) << 3 as libc::c_int |
                          (1 as libc::c_int) << 8 as libc::c_int |
                          (1 as libc::c_int) << 15 as libc::c_int) -
                         (((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int) >>
                              1 as libc::c_int & 0x77777777 as libc::c_int) -
                         (((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int) >>
                              2 as libc::c_int & 0x33333333 as libc::c_int) -
                         (((1 as libc::c_int) << 1 as libc::c_int |
                               (1 as libc::c_int) << 2 as libc::c_int |
                               (1 as libc::c_int) << 3 as libc::c_int |
                               (1 as libc::c_int) << 8 as libc::c_int |
                               (1 as libc::c_int) << 15 as libc::c_int) >>
                              3 as libc::c_int & 0x11111111 as libc::c_int) >>
                         4 as libc::c_int) & 0xf0f0f0f as libc::c_int) %
                   255 as libc::c_int) as libc::c_uint {
        timerInfo[i_1 as usize].priority = !(0 as libc::c_int) as uint8_t;
        i_1 = i_1.wrapping_add(1)
    };
}
// finish configuring timers after allocation phase
// start timers
// TODO - Work in progress - initialization routine must be modified/verified to start correctly without timers
#[no_mangle]
pub unsafe extern "C" fn timerStart() { }
/* *
 * Force an overflow for a given timer.
 * Saves the current value of the counter in the relevant timerConfig's forcedOverflowTimerValue variable.
 * @param TIM_Typedef *tim The timer to overflow
 * @return void
 **/
#[no_mangle]
pub unsafe extern "C" fn timerForceOverflow(mut tim: *mut TIM_TypeDef) {
    let mut timerIndex: uint8_t = lookupTimerIndex(tim as *const TIM_TypeDef);
    let mut __basepri_save: uint8_t = __get_BASEPRI() as uint8_t;
    let mut __ToDo: uint8_t =
        __basepriSetMemRetVal((((1 as libc::c_int) <<
                                    (4 as libc::c_int as
                                         libc::c_uint).wrapping_sub((7 as
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
                                   << 4 as libc::c_int & 0xf0 as libc::c_int)
                                  as uint8_t);
    while __ToDo != 0 {
        // Save the current count so that PPM reading will work on the same timer that was forced to overflow
        timerConfig[timerIndex as usize].forcedOverflowTimerValue =
            (*tim).CNT.wrapping_add(1 as libc::c_int as libc::c_uint);
        // Force an overflow by setting the UG bit
        ::core::ptr::write_volatile(&mut (*tim).EGR as *mut uint32_t,
                                    (::core::ptr::read_volatile::<uint32_t>(&(*tim).EGR
                                                                                as
                                                                                *const uint32_t)
                                         as libc::c_uint |
                                         0x1 as libc::c_int as uint16_t as
                                             libc::c_uint) as uint32_t as
                                        uint32_t);
        __ToDo = 0 as libc::c_int as uint8_t
    };
}
#[no_mangle]
pub unsafe extern "C" fn timerOCInit(mut tim: *mut TIM_TypeDef,
                                     mut channel: uint8_t,
                                     mut init: *mut TIM_OCInitTypeDef) {
    match channel as libc::c_int {
        0 => { TIM_OC1Init(tim, init); }
        4 => { TIM_OC2Init(tim, init); }
        8 => { TIM_OC3Init(tim, init); }
        12 => { TIM_OC4Init(tim, init); }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn timerOCPreloadConfig(mut tim: *mut TIM_TypeDef,
                                              mut channel: uint8_t,
                                              mut preload: uint16_t) {
    match channel as libc::c_int {
        0 => { TIM_OC1PreloadConfig(tim, preload); }
        4 => { TIM_OC2PreloadConfig(tim, preload); }
        8 => { TIM_OC3PreloadConfig(tim, preload); }
        12 => { TIM_OC4PreloadConfig(tim, preload); }
        _ => { }
    };
}
#[no_mangle]
pub unsafe extern "C" fn timerCCR(mut tim: *mut TIM_TypeDef,
                                  mut channel: uint8_t) -> *mut timCCR_t {
    return (&mut (*tim).CCR1 as *mut uint32_t as
                *mut libc::c_char).offset(channel as libc::c_int as isize) as
               *mut timCCR_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerDmaSource(mut channel: uint8_t) -> uint16_t {
    match channel as libc::c_int {
        0 => { return 0x200 as libc::c_int as uint16_t }
        4 => { return 0x400 as libc::c_int as uint16_t }
        8 => { return 0x800 as libc::c_int as uint16_t }
        12 => { return 0x1000 as libc::c_int as uint16_t }
        _ => { }
    }
    return 0 as libc::c_int as uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerGetPrescalerByDesiredMhz(mut tim:
                                                           *mut TIM_TypeDef,
                                                       mut mhz: uint16_t)
 -> uint16_t {
    return timerGetPrescalerByDesiredHertz(tim,
                                           (mhz as libc::c_int *
                                                1000000 as libc::c_int) as
                                               uint32_t);
}
#[no_mangle]
pub unsafe extern "C" fn timerGetPeriodByPrescaler(mut tim: *mut TIM_TypeDef,
                                                   mut prescaler: uint16_t,
                                                   mut hz: uint32_t)
 -> uint16_t {
    return timerClock(tim).wrapping_div((prescaler as libc::c_int +
                                             1 as libc::c_int) as
                                            libc::c_uint).wrapping_div(hz) as
               uint16_t;
}
#[no_mangle]
pub unsafe extern "C" fn timerGetPrescalerByDesiredHertz(mut tim:
                                                             *mut TIM_TypeDef,
                                                         mut hz: uint32_t)
 -> uint16_t {
    // protection here for desired hertz > SystemCoreClock???
    if hz > timerClock(tim) { return 0 as libc::c_int as uint16_t }
    return (timerClock(tim).wrapping_add(hz.wrapping_div(2 as libc::c_int as
                                                             libc::c_uint)).wrapping_div(hz)
                as uint16_t as libc::c_int - 1 as libc::c_int) as uint16_t;
}
unsafe extern "C" fn run_static_initializers() {
    usedTimers =
        [(0x40000000 as libc::c_int as
              uint32_t).wrapping_add(0x10000 as libc::c_int as
                                         libc::c_uint).wrapping_add(0x2c00 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
             as *mut TIM_TypeDef,
         (0x40000000 as libc::c_int as
              uint32_t).wrapping_add(0 as libc::c_int as libc::c_uint) as
             *mut TIM_TypeDef,
         (0x40000000 as libc::c_int as
              uint32_t).wrapping_add(0x400 as libc::c_int as libc::c_uint) as
             *mut TIM_TypeDef,
         (0x40000000 as libc::c_int as
              uint32_t).wrapping_add(0x10000 as libc::c_int as
                                         libc::c_uint).wrapping_add(0x3400 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
             as *mut TIM_TypeDef,
         (0x40000000 as libc::c_int as
              uint32_t).wrapping_add(0x10000 as libc::c_int as
                                         libc::c_uint).wrapping_add(0x4000 as
                                                                        libc::c_int
                                                                        as
                                                                        libc::c_uint)
             as *mut TIM_TypeDef]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
