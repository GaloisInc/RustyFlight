use ::libc;
extern "C" {
    #[no_mangle]
    fn RCC_AHBPeriphClockCmd(RCC_AHBPeriph: uint32_t,
                             NewState: FunctionalState);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
}
pub type __uint8_t = libc::c_uchar;
pub type __uint32_t = libc::c_uint;
pub type uint8_t = __uint8_t;
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
pub struct DMA_TypeDef {
    pub ISR: uint32_t,
    pub IFCR: uint32_t,
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
/*
 * DMA descriptors.
 */
// Initialized in run_static_initializers
static mut dmaDescriptors: [dmaChannelDescriptor_t; 12] =
    [dmaChannelDescriptor_t{dma: 0 as *mut DMA_TypeDef,
                            ref_0: 0 as *mut DMA_Channel_TypeDef,
                            irqHandlerCallback: None,
                            flagsShift: 0,
                            irqN: WWDG_IRQn,
                            userParam: 0,
                            owner: OWNER_FREE,
                            resourceIndex: 0,
                            completeFlag: 0,}; 12];
/*
 * DMA IRQ Handlers
 */
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel1_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH1_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel2_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH2_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel3_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH3_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel4_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH4_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel5_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH5_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel6_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH6_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA1_Channel7_IRQHandler() {
    let index: uint8_t =
        (DMA1_CH7_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Channel1_IRQHandler() {
    let index: uint8_t =
        (DMA2_CH1_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Channel2_IRQHandler() {
    let index: uint8_t =
        (DMA2_CH2_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Channel3_IRQHandler() {
    let index: uint8_t =
        (DMA2_CH3_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Channel4_IRQHandler() {
    let index: uint8_t =
        (DMA2_CH4_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn DMA2_Channel5_IRQHandler() {
    let index: uint8_t =
        (DMA2_CH5_HANDLER as libc::c_int - 1 as libc::c_int) as uint8_t;
    if dmaDescriptors[index as usize].irqHandlerCallback.is_some() {
        dmaDescriptors[index as
                           usize].irqHandlerCallback.expect("non-null function pointer")(&mut *dmaDescriptors.as_mut_ptr().offset(index
                                                                                                                                      as
                                                                                                                                      isize));
    };
}
#[no_mangle]
pub unsafe extern "C" fn dmaFlag_IT_TCIF(mut channel:
                                             *const DMA_Channel_TypeDef)
 -> uint32_t {
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x8 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x2 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x1c as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x20 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x30 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x200 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x44 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x2000 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x58 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x20000 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x6c as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x200000 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x80 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x2000000 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x408 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x10000002 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x41c as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x10000020 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x430 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x10000200 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x444 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x10002000 as libc::c_int as uint32_t
    }
    if channel ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x20000 as libc::c_int as
                                           libc::c_uint).wrapping_add(0x458 as
                                                                          libc::c_int
                                                                          as
                                                                          libc::c_uint)
               as *mut DMA_Channel_TypeDef as *const DMA_Channel_TypeDef {
        return 0x10020000 as libc::c_int as uint32_t
    }
    return 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn dmaInit(mut identifier: dmaIdentifier_e,
                                 mut owner: resourceOwner_e,
                                 mut resourceIndex: uint8_t) {
    let index: libc::c_int =
        (identifier as
             libc::c_uint).wrapping_sub(1 as libc::c_int as libc::c_uint) as
            libc::c_int;
    RCC_AHBPeriphClockCmd(if dmaDescriptors[index as usize].dma ==
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x20000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut DMA_TypeDef {
                              0x1 as libc::c_int as uint32_t
                          } else { 0x2 as libc::c_int as uint32_t }, ENABLE);
    dmaDescriptors[index as usize].owner = owner;
    dmaDescriptors[index as usize].resourceIndex = resourceIndex;
}
#[no_mangle]
pub unsafe extern "C" fn dmaSetHandler(mut identifier: dmaIdentifier_e,
                                       mut callback:
                                           dmaCallbackHandlerFuncPtr,
                                       mut priority: uint32_t,
                                       mut userParam: uint32_t) {
    let mut NVIC_InitStructure: NVIC_InitTypeDef =
        NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                         NVIC_IRQChannelPreemptionPriority: 0,
                         NVIC_IRQChannelSubPriority: 0,
                         NVIC_IRQChannelCmd: DISABLE,};
    let index: libc::c_int =
        (identifier as
             libc::c_uint).wrapping_sub(1 as libc::c_int as libc::c_uint) as
            libc::c_int;
    /* TODO: remove this - enforce the init */
    RCC_AHBPeriphClockCmd(if dmaDescriptors[index as usize].dma ==
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x20000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut DMA_TypeDef {
                              0x1 as libc::c_int as uint32_t
                          } else { 0x2 as libc::c_int as uint32_t }, ENABLE);
    dmaDescriptors[index as usize].irqHandlerCallback = callback;
    dmaDescriptors[index as usize].userParam = userParam;
    dmaDescriptors[index as usize].completeFlag =
        dmaFlag_IT_TCIF(dmaDescriptors[index as usize].ref_0);
    NVIC_InitStructure.NVIC_IRQChannel =
        dmaDescriptors[index as usize].irqN as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =
        (priority >>
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
        ((priority &
              (0xf as libc::c_int >>
                   (7 as libc::c_int as
                        libc::c_uint).wrapping_sub(0x500 as libc::c_int as
                                                       uint32_t >>
                                                       8 as libc::c_int)) as
                  libc::c_uint) >> 4 as libc::c_int) as uint8_t;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&mut NVIC_InitStructure);
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetOwner(mut identifier: dmaIdentifier_e)
 -> resourceOwner_e {
    return dmaDescriptors[(identifier as
                               libc::c_uint).wrapping_sub(1 as libc::c_int as
                                                              libc::c_uint) as
                              usize].owner;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetResourceIndex(mut identifier: dmaIdentifier_e)
 -> uint8_t {
    return dmaDescriptors[(identifier as
                               libc::c_uint).wrapping_sub(1 as libc::c_int as
                                                              libc::c_uint) as
                              usize].resourceIndex;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetIdentifier(mut channel:
                                              *const DMA_Channel_TypeDef)
 -> dmaIdentifier_e {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < DMA_LAST_HANDLER as libc::c_int {
        if dmaDescriptors[i as usize].ref_0 ==
               channel as *mut DMA_Channel_TypeDef {
            return (i + 1 as libc::c_int) as dmaIdentifier_e
        }
        i += 1
    }
    return DMA_NONE;
}
#[no_mangle]
pub unsafe extern "C" fn dmaGetRefByIdentifier(identifier: dmaIdentifier_e)
 -> *mut DMA_Channel_TypeDef {
    return dmaDescriptors[(identifier as
                               libc::c_uint).wrapping_sub(1 as libc::c_int as
                                                              libc::c_uint) as
                              usize].ref_0;
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
pub unsafe extern "C" fn dmaGetDescriptorByIdentifier(identifier:
                                                          dmaIdentifier_e)
 -> *mut dmaChannelDescriptor_t {
    return &mut *dmaDescriptors.as_mut_ptr().offset((identifier as
                                                         libc::c_uint).wrapping_sub(1
                                                                                        as
                                                                                        libc::c_int
                                                                                        as
                                                                                        libc::c_uint)
                                                        as isize) as
               *mut dmaChannelDescriptor_t;
}
unsafe extern "C" fn run_static_initializers() {
    dmaDescriptors =
        [{
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x8
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            0 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel1_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x1c
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            4 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel2_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x30
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            8 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel3_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x44
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            12 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel4_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x58
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            16 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel5_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x6c
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            20 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel6_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x80
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            24 as libc::c_int as uint8_t,
                                        irqN: DMA1_Channel7_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x408
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            0 as libc::c_int as uint8_t,
                                        irqN: DMA2_Channel1_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x41c
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            4 as libc::c_int as uint8_t,
                                        irqN: DMA2_Channel2_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x430
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            8 as libc::c_int as uint8_t,
                                        irqN: DMA2_Channel3_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x444
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            12 as libc::c_int as uint8_t,
                                        irqN: DMA2_Channel4_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         },
         {
             let mut init =
                 dmaChannelDescriptor_s{dma:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x400
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_TypeDef,
                                        ref_0:
                                            (0x40000000 as libc::c_int as
                                                 uint32_t).wrapping_add(0x20000
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint).wrapping_add(0x458
                                                                                                           as
                                                                                                           libc::c_int
                                                                                                           as
                                                                                                           libc::c_uint)
                                                as *mut DMA_Channel_TypeDef,
                                        irqHandlerCallback: None,
                                        flagsShift:
                                            16 as libc::c_int as uint8_t,
                                        irqN: DMA2_Channel5_IRQn,
                                        userParam:
                                            0 as libc::c_int as uint32_t,
                                        owner: OWNER_FREE,
                                        resourceIndex:
                                            0 as libc::c_int as uint8_t,
                                        completeFlag: 0,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
