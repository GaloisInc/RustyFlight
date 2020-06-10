use ::libc;
extern "C" {
    #[no_mangle]
    fn I2C_Init(I2Cx: *mut I2C_TypeDef, I2C_InitStruct: *mut I2C_InitTypeDef);
    #[no_mangle]
    fn I2C_StructInit(I2C_InitStruct: *mut I2C_InitTypeDef);
    #[no_mangle]
    fn I2C_Cmd(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_SoftwareResetCmd(I2Cx: *mut I2C_TypeDef);
    #[no_mangle]
    fn I2C_ITConfig(I2Cx: *mut I2C_TypeDef, I2C_IT: uint32_t,
                    NewState: FunctionalState);
    #[no_mangle]
    fn I2C_GeneralCallCmd(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_GenerateSTOP(I2Cx: *mut I2C_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn I2C_GetAddressMatched(I2Cx: *mut I2C_TypeDef) -> uint8_t;
    #[no_mangle]
    fn I2C_GetTransferDirection(I2Cx: *mut I2C_TypeDef) -> uint16_t;
    #[no_mangle]
    fn I2C_TransferHandling(I2Cx: *mut I2C_TypeDef, Address: uint16_t,
                            Number_Bytes: uint8_t, ReloadEndMode: uint32_t,
                            StartStopMode: uint32_t);
    /* Data transfers management functions ****************************************/
    #[no_mangle]
    fn I2C_SendData(I2Cx: *mut I2C_TypeDef, Data: uint8_t);
    #[no_mangle]
    fn I2C_ReceiveData(I2Cx: *mut I2C_TypeDef) -> uint8_t;
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn I2C_GetFlagStatus(I2Cx: *mut I2C_TypeDef, I2C_FLAG: uint32_t)
     -> FlagStatus;
    #[no_mangle]
    fn I2C_GetITStatus(I2Cx: *mut I2C_TypeDef, I2C_IT: uint32_t) -> ITStatus;
    #[no_mangle]
    fn I2C_ClearITPendingBit(I2Cx: *mut I2C_TypeDef, I2C_IT: uint32_t);
    #[no_mangle]
    fn RCC_I2CCLKConfig(RCC_I2CCLK: uint32_t);
    #[no_mangle]
    fn NVIC_Init(NVIC_InitStruct: *mut NVIC_InitTypeDef);
    // unimplemented
    #[no_mangle]
    fn IOConfigGPIOAF(io: IO_t, cfg: ioConfig_t, af: uint8_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IO_GPIO(io: IO_t) -> *mut GPIO_TypeDef;
    #[no_mangle]
    fn IO_Pin(io: IO_t) -> uint16_t;
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    fn micros() -> uint32_t;
    #[no_mangle]
    fn bstProcessInCommand();
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
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type ITStatus = FlagStatus;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
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
  * @brief Inter-integrated Circuit Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_TypeDef {
    pub CR1: uint32_t,
    pub CR2: uint32_t,
    pub OAR1: uint32_t,
    pub OAR2: uint32_t,
    pub TIMINGR: uint32_t,
    pub TIMEOUTR: uint32_t,
    pub ISR: uint32_t,
    pub ICR: uint32_t,
    pub PECR: uint32_t,
    pub RXDR: uint32_t,
    pub TXDR: uint32_t,
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
/* *
  ******************************************************************************
  * @file    stm32f30x_i2c.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the I2C firmware
  *          library.
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
/* * @addtogroup I2C
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* *
  * @brief  I2C Init structure definition
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct I2C_InitTypeDef {
    pub I2C_Timing: uint32_t,
    pub I2C_AnalogFilter: uint32_t,
    pub I2C_DigitalFilter: uint32_t,
    pub I2C_Mode: uint32_t,
    pub I2C_OwnAddress1: uint32_t,
    pub I2C_Ack: uint32_t,
    pub I2C_AcknowledgedAddress: uint32_t,
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
pub type BSTDevice = libc::c_uint;
pub const BSTDEV_2: BSTDevice = 1;
pub const BSTDEV_1: BSTDevice = 0;
/* PF6 */
static mut bstErrorCount: uint16_t = 0 as libc::c_int as uint16_t;
static mut BSTx: *mut I2C_TypeDef =
    0 as *const I2C_TypeDef as *mut I2C_TypeDef;
static mut scl: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut sda: IO_t = 0 as *const libc::c_void as *mut libc::c_void;
static mut bstSclGpio: *mut GPIO_TypeDef =
    0 as *const GPIO_TypeDef as *mut GPIO_TypeDef;
static mut bstSclPin: uint16_t = 0;
#[no_mangle]
pub static mut CRC8: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut coreProReady: bool = 0 as libc::c_int != 0;
// /////////////////////////////////////////////////////////////////////////////
// BST TimeoutUserCallback
// /////////////////////////////////////////////////////////////////////////////
#[no_mangle]
pub static mut dataBuffer: [uint8_t; 128] =
    [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
#[no_mangle]
pub static mut dataBufferPointer: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut bstWriteDataLen: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut writeData: [uint8_t; 128] =
    [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
#[no_mangle]
pub static mut currentWriteBufferPointer: uint8_t =
    0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut receiverAddress: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut readData: [uint8_t; 128] =
    [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
#[no_mangle]
pub static mut bufferPointer: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub static mut cleanflight_data_ready: bool = 0 as libc::c_int != 0;
#[no_mangle]
pub static mut interruptCounter: uint8_t = 0 as libc::c_int as uint8_t;
#[no_mangle]
pub unsafe extern "C" fn I2C_EV_IRQHandler() {
    if I2C_GetITStatus(BSTx, 0x8 as libc::c_int as uint32_t) as u64 != 0 {
        ::core::ptr::write_volatile(&mut CRC8 as *mut uint8_t,
                                    0 as libc::c_int as
                                        uint8_t); // 100 Khz, 72Mhz Clock, Analog Filter Delay ON, Rise 100, Fall 10.
        if I2C_GetTransferDirection(BSTx) as libc::c_int ==
               0x400 as libc::c_int as uint16_t as libc::c_int {
            currentWriteBufferPointer = 0 as libc::c_int as uint8_t;
            receiverAddress = 1 as libc::c_int != 0;
            let fresh0 = currentWriteBufferPointer;
            currentWriteBufferPointer =
                currentWriteBufferPointer.wrapping_add(1);
            I2C_SendData(BSTx, writeData[fresh0 as usize]);
            I2C_ITConfig(BSTx, 0x2 as libc::c_int as uint32_t, ENABLE);
        } else {
            readData[0 as libc::c_int as usize] = I2C_GetAddressMatched(BSTx);
            bufferPointer = 1 as libc::c_int as uint8_t
        }
        I2C_ClearITPendingBit(BSTx, 0x8 as libc::c_int as uint32_t);
    } else if I2C_GetITStatus(BSTx, 0x4 as libc::c_int as uint32_t) as u64 !=
                  0 {
        let mut data: uint8_t = I2C_ReceiveData(BSTx);
        readData[bufferPointer as usize] = data;
        if bufferPointer as libc::c_int > 1 as libc::c_int {
            if readData[1 as libc::c_int as usize] as libc::c_int +
                   1 as libc::c_int == bufferPointer as libc::c_int {
                crc8Cal(0 as libc::c_int as uint8_t);
                bstProcessInCommand();
            } else { crc8Cal(data); }
        }
        bufferPointer = bufferPointer.wrapping_add(1);
        I2C_ClearITPendingBit(BSTx, 0x4 as libc::c_int as uint32_t);
    } else if I2C_GetITStatus(BSTx, 0x2 as libc::c_int as uint32_t) as u64 !=
                  0 {
        if receiverAddress {
            if currentWriteBufferPointer as libc::c_int > 0 as libc::c_int {
                if !cleanflight_data_ready {
                    I2C_ClearITPendingBit(BSTx,
                                          0x2 as libc::c_int as uint32_t);
                    return
                }
                if (interruptCounter as libc::c_int) < 40 as libc::c_int {
                    interruptCounter = interruptCounter.wrapping_add(1);
                    I2C_ClearITPendingBit(BSTx,
                                          0x2 as libc::c_int as uint32_t);
                    return
                } else { interruptCounter = 0 as libc::c_int as uint8_t }
                if writeData[0 as libc::c_int as usize] as libc::c_int ==
                       currentWriteBufferPointer as libc::c_int {
                    receiverAddress = 0 as libc::c_int != 0;
                    crc8Cal(0 as libc::c_int as uint8_t);
                    I2C_SendData(BSTx, CRC8);
                    I2C_ITConfig(BSTx, 0x2 as libc::c_int as uint32_t,
                                 DISABLE);
                } else {
                    crc8Cal(writeData[currentWriteBufferPointer as usize]);
                    let fresh1 = currentWriteBufferPointer;
                    currentWriteBufferPointer =
                        currentWriteBufferPointer.wrapping_add(1);
                    I2C_SendData(BSTx, writeData[fresh1 as usize]);
                }
            }
        } else if bstWriteDataLen != 0 {
            I2C_SendData(BSTx, dataBuffer[dataBufferPointer as usize]);
            if bstWriteDataLen as libc::c_int > 1 as libc::c_int {
                dataBufferPointer = dataBufferPointer.wrapping_add(1)
            }
            if dataBufferPointer as libc::c_int ==
                   bstWriteDataLen as libc::c_int {
                I2C_ITConfig(BSTx, 0x2 as libc::c_int as uint32_t, DISABLE);
                dataBufferPointer = 0 as libc::c_int as uint8_t;
                bstWriteDataLen = 0 as libc::c_int as uint8_t
            }
        }
        I2C_ClearITPendingBit(BSTx, 0x2 as libc::c_int as uint32_t);
    } else if I2C_GetITStatus(BSTx, 0x10 as libc::c_int as uint32_t) as u64 !=
                  0 {
        if receiverAddress {
            receiverAddress = 0 as libc::c_int != 0;
            I2C_ITConfig(BSTx, 0x2 as libc::c_int as uint32_t, DISABLE);
        }
        I2C_ClearITPendingBit(BSTx, 0x10 as libc::c_int as uint32_t);
    } else if I2C_GetITStatus(BSTx, 0x20 as libc::c_int as uint32_t) as u64 !=
                  0 {
        if bstWriteDataLen as libc::c_int != 0 &&
               dataBufferPointer as libc::c_int ==
                   bstWriteDataLen as libc::c_int {
            dataBufferPointer = 0 as libc::c_int as uint8_t;
            bstWriteDataLen = 0 as libc::c_int as uint8_t
        }
        I2C_ClearITPendingBit(BSTx, 0x20 as libc::c_int as uint32_t);
    } else if I2C_GetITStatus(BSTx, 0x100 as libc::c_int as uint32_t) as
                  libc::c_uint != 0 ||
                  I2C_GetITStatus(BSTx, 0x200 as libc::c_int as uint32_t) as
                      libc::c_uint != 0 ||
                  I2C_GetITStatus(BSTx, 0x400 as libc::c_int as uint32_t) as
                      libc::c_uint != 0 {
        bstTimeoutUserCallback();
        I2C_ClearITPendingBit(BSTx,
                              0x100 as libc::c_int as uint32_t |
                                  0x200 as libc::c_int as uint32_t |
                                  0x400 as libc::c_int as uint32_t);
    };
}
#[no_mangle]
pub unsafe extern "C" fn I2C1_EV_IRQHandler() { I2C_EV_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn I2C2_EV_IRQHandler() { I2C_EV_IRQHandler(); }
#[no_mangle]
pub unsafe extern "C" fn bstTimeoutUserCallback() -> uint32_t {
    ::core::ptr::write_volatile(&mut bstErrorCount as *mut uint16_t,
                                ::core::ptr::read_volatile::<uint16_t>(&bstErrorCount
                                                                           as
                                                                           *const uint16_t).wrapping_add(1));
    I2C_GenerateSTOP(BSTx, ENABLE);
    receiverAddress = 0 as libc::c_int != 0;
    dataBufferPointer = 0 as libc::c_int as uint8_t;
    bstWriteDataLen = 0 as libc::c_int as uint8_t;
    I2C_ITConfig(BSTx, 0x2 as libc::c_int as uint32_t, DISABLE);
    I2C_SoftwareResetCmd(BSTx);
    return 0 as libc::c_int as uint32_t;
}
#[no_mangle]
pub unsafe extern "C" fn bstInit(mut index: BSTDevice) {
    let mut nvic: NVIC_InitTypeDef =
        NVIC_InitTypeDef{NVIC_IRQChannel: 0,
                         NVIC_IRQChannelPreemptionPriority: 0,
                         NVIC_IRQChannelSubPriority: 0,
                         NVIC_IRQChannelCmd: DISABLE,};
    let mut bstInit_0: I2C_InitTypeDef =
        I2C_InitTypeDef{I2C_Timing: 0,
                        I2C_AnalogFilter: 0,
                        I2C_DigitalFilter: 0,
                        I2C_Mode: 0,
                        I2C_OwnAddress1: 0,
                        I2C_Ack: 0,
                        I2C_AcknowledgedAddress: 0,};
    BSTx =
        if index as libc::c_uint == BSTDEV_1 as libc::c_int as libc::c_uint {
            (0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x5400 as libc::c_int as libc::c_uint)
                as *mut I2C_TypeDef
        } else {
            (0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x5800 as libc::c_int as libc::c_uint)
                as *mut I2C_TypeDef
        };
    if BSTx ==
           (0x40000000 as libc::c_int as
                uint32_t).wrapping_add(0x5400 as libc::c_int as libc::c_uint)
               as *mut I2C_TypeDef {
        scl =
            IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) <<
                            4 as libc::c_int | 6 as libc::c_int) as ioTag_t);
        sda =
            IOGetByTag(((1 as libc::c_int + 1 as libc::c_int) <<
                            4 as libc::c_int | 7 as libc::c_int) as ioTag_t)
    } else {
        scl =
            IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                            4 as libc::c_int | 9 as libc::c_int) as ioTag_t);
        sda =
            IOGetByTag(((0 as libc::c_int + 1 as libc::c_int) <<
                            4 as libc::c_int | 10 as libc::c_int) as ioTag_t)
    }
    bstSclGpio = IO_GPIO(scl);
    bstSclPin = IO_Pin(scl);
    RCC_ClockCmd(if BSTx ==
                        (0x40000000 as libc::c_int as
                             uint32_t).wrapping_add(0x5800 as libc::c_int as
                                                        libc::c_uint) as
                            *mut I2C_TypeDef {
                     (((RCC_APB1 as libc::c_int) << 5 as libc::c_int) as
                          libc::c_long) |
                         (16 as libc::c_int *
                              (0x400000 as libc::c_int as uint32_t as
                                   libc::c_long > 65535 as libc::c_long) as
                                  libc::c_int) as libc::c_long +
                             ((8 as libc::c_int *
                                   (0x400000 as libc::c_int as uint32_t as
                                        libc::c_long * 1 as libc::c_long >>
                                        16 as libc::c_int *
                                            (0x400000 as libc::c_int as
                                                 uint32_t as libc::c_long >
                                                 65535 as libc::c_long) as
                                                libc::c_int >
                                        255 as libc::c_int as libc::c_long) as
                                       libc::c_int) as libc::c_long +
                                  (8 as libc::c_int as libc::c_long -
                                       90 as libc::c_int as libc::c_long /
                                           ((0x400000 as libc::c_int as
                                                 uint32_t as libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (0x400000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 8 as libc::c_int *
                                                     (0x400000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (0x400000 as
                                                                   libc::c_int
                                                                   as uint32_t
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >
                                                          255 as libc::c_int
                                                              as libc::c_long)
                                                         as libc::c_int) /
                                                4 as libc::c_int as
                                                    libc::c_long +
                                                14 as libc::c_int as
                                                    libc::c_long |
                                                1 as libc::c_int as
                                                    libc::c_long) -
                                       2 as libc::c_int as libc::c_long /
                                           ((0x400000 as libc::c_int as
                                                 uint32_t as libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (0x400000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 8 as libc::c_int *
                                                     (0x400000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (0x400000 as
                                                                   libc::c_int
                                                                   as uint32_t
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >
                                                          255 as libc::c_int
                                                              as libc::c_long)
                                                         as libc::c_int) /
                                                2 as libc::c_int as
                                                    libc::c_long +
                                                1 as libc::c_int as
                                                    libc::c_long)))
                 } else {
                     (((RCC_APB1 as libc::c_int) << 5 as libc::c_int) as
                          libc::c_long) |
                         (16 as libc::c_int *
                              (0x200000 as libc::c_int as uint32_t as
                                   libc::c_long > 65535 as libc::c_long) as
                                  libc::c_int) as libc::c_long +
                             ((8 as libc::c_int *
                                   (0x200000 as libc::c_int as uint32_t as
                                        libc::c_long * 1 as libc::c_long >>
                                        16 as libc::c_int *
                                            (0x200000 as libc::c_int as
                                                 uint32_t as libc::c_long >
                                                 65535 as libc::c_long) as
                                                libc::c_int >
                                        255 as libc::c_int as libc::c_long) as
                                       libc::c_int) as libc::c_long +
                                  (8 as libc::c_int as libc::c_long -
                                       90 as libc::c_int as libc::c_long /
                                           ((0x200000 as libc::c_int as
                                                 uint32_t as libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (0x200000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 8 as libc::c_int *
                                                     (0x200000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (0x200000 as
                                                                   libc::c_int
                                                                   as uint32_t
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >
                                                          255 as libc::c_int
                                                              as libc::c_long)
                                                         as libc::c_int) /
                                                4 as libc::c_int as
                                                    libc::c_long +
                                                14 as libc::c_int as
                                                    libc::c_long |
                                                1 as libc::c_int as
                                                    libc::c_long) -
                                       2 as libc::c_int as libc::c_long /
                                           ((0x200000 as libc::c_int as
                                                 uint32_t as libc::c_long *
                                                 1 as libc::c_long >>
                                                 16 as libc::c_int *
                                                     (0x200000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long >
                                                          65535 as
                                                              libc::c_long) as
                                                         libc::c_int >>
                                                 8 as libc::c_int *
                                                     (0x200000 as libc::c_int
                                                          as uint32_t as
                                                          libc::c_long *
                                                          1 as libc::c_long >>
                                                          16 as libc::c_int *
                                                              (0x200000 as
                                                                   libc::c_int
                                                                   as uint32_t
                                                                   as
                                                                   libc::c_long
                                                                   >
                                                                   65535 as
                                                                       libc::c_long)
                                                                  as
                                                                  libc::c_int
                                                          >
                                                          255 as libc::c_int
                                                              as libc::c_long)
                                                         as libc::c_int) /
                                                2 as libc::c_int as
                                                    libc::c_long +
                                                1 as libc::c_int as
                                                    libc::c_long)))
                 } as rccPeriphTag_t, ENABLE);
    RCC_I2CCLKConfig(if BSTx ==
                            (0x40000000 as libc::c_int as
                                 uint32_t).wrapping_add(0x5800 as libc::c_int
                                                            as libc::c_uint)
                                as *mut I2C_TypeDef {
                         0x10000020 as libc::c_int as uint32_t
                     } else { 0x10 as libc::c_int as uint32_t });
    IOInit(scl, OWNER_I2C_SCL,
           (index as
                libc::c_uint).wrapping_add(1 as libc::c_int as libc::c_uint)
               as uint8_t);
    IOConfigGPIOAF(scl,
                   (GPIO_Mode_AF as libc::c_int |
                        (GPIO_Speed_Level_3 as libc::c_int) <<
                            2 as libc::c_int |
                        (GPIO_OType_OD as libc::c_int) << 4 as libc::c_int |
                        (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                       as ioConfig_t, 0x4 as libc::c_int as uint8_t);
    IOInit(sda, OWNER_I2C_SDA,
           (index as
                libc::c_uint).wrapping_add(1 as libc::c_int as libc::c_uint)
               as uint8_t);
    IOConfigGPIOAF(sda,
                   (GPIO_Mode_AF as libc::c_int |
                        (GPIO_Speed_Level_3 as libc::c_int) <<
                            2 as libc::c_int |
                        (GPIO_OType_OD as libc::c_int) << 4 as libc::c_int |
                        (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                       as ioConfig_t, 0x4 as libc::c_int as uint8_t);
    I2C_StructInit(&mut bstInit_0);
    bstInit_0.I2C_Mode = 0 as libc::c_int as uint32_t;
    bstInit_0.I2C_AnalogFilter = 0 as libc::c_int as uint32_t;
    bstInit_0.I2C_DigitalFilter = 0 as libc::c_int as uint32_t;
    bstInit_0.I2C_OwnAddress1 = 0xc8 as libc::c_int as uint32_t;
    bstInit_0.I2C_Ack = 0 as libc::c_int as uint32_t;
    bstInit_0.I2C_AcknowledgedAddress = 0 as libc::c_int as uint32_t;
    bstInit_0.I2C_Timing = 0x30e0257a as libc::c_int as uint32_t;
    I2C_Init(BSTx, &mut bstInit_0);
    I2C_GeneralCallCmd(BSTx, ENABLE);
    nvic.NVIC_IRQChannel =
        if BSTx ==
               (0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x5800 as libc::c_int as
                                               libc::c_uint) as
                   *mut I2C_TypeDef {
            I2C2_EV_IRQn as libc::c_int
        } else { I2C1_EV_IRQn as libc::c_int } as uint8_t;
    nvic.NVIC_IRQChannelPreemptionPriority =
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
    nvic.NVIC_IRQChannelSubPriority =
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
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&mut nvic);
    I2C_ITConfig(BSTx,
                 0x8 as libc::c_int as uint32_t |
                     0x4 as libc::c_int as uint32_t |
                     0x20 as libc::c_int as uint32_t |
                     0x10 as libc::c_int as uint32_t |
                     0x80 as libc::c_int as uint32_t, ENABLE);
    I2C_Cmd(BSTx, ENABLE);
}
#[no_mangle]
pub unsafe extern "C" fn bstGetErrorCounter() -> uint16_t {
    return bstErrorCount;
}
/* ************************************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn bstWriteBusy() -> bool {
    if bstWriteDataLen != 0 {
        return 1 as libc::c_int != 0
    } else { return 0 as libc::c_int != 0 };
}
#[no_mangle]
pub unsafe extern "C" fn bstMasterWrite(mut data: *mut uint8_t) -> bool {
    if bstWriteDataLen as libc::c_int == 0 as libc::c_int {
        ::core::ptr::write_volatile(&mut CRC8 as *mut uint8_t,
                                    0 as libc::c_int as uint8_t);
        dataBufferPointer = 0 as libc::c_int as uint8_t;
        dataBuffer[0 as libc::c_int as usize] = *data;
        dataBuffer[1 as libc::c_int as usize] =
            *data.offset(1 as libc::c_int as isize);
        bstWriteDataLen =
            (dataBuffer[1 as libc::c_int as usize] as libc::c_int +
                 2 as libc::c_int) as uint8_t;
        let mut i: uint8_t = 2 as libc::c_int as uint8_t;
        while (i as libc::c_int) < bstWriteDataLen as libc::c_int {
            if i as libc::c_int ==
                   bstWriteDataLen as libc::c_int - 1 as libc::c_int {
                crc8Cal(0 as libc::c_int as uint8_t);
                dataBuffer[i as usize] = CRC8
            } else {
                dataBuffer[i as usize] =
                    *data.offset(i as libc::c_int as isize);
                crc8Cal(dataBuffer[i as usize]);
            }
            i = i.wrapping_add(1)
        }
        return 1 as libc::c_int != 0
    }
    return 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn bstMasterWriteLoop() {
    static mut bstMasterWriteTimeout: uint32_t = 0 as libc::c_int as uint32_t;
    let mut currentTime: uint32_t = micros();
    if bstWriteDataLen as libc::c_int != 0 &&
           dataBufferPointer as libc::c_int == 0 as libc::c_int {
        let mut scl_set: bool =
            (*bstSclGpio).IDR as libc::c_int & bstSclPin as libc::c_int != 0;
        if I2C_GetFlagStatus(BSTx, 0x8000 as libc::c_int as uint32_t) as
               libc::c_uint == RESET as libc::c_int as libc::c_uint &&
               scl_set as libc::c_int != 0 {
            I2C_TransferHandling(BSTx,
                                 dataBuffer[dataBufferPointer as usize] as
                                     uint16_t,
                                 (dataBuffer[(dataBufferPointer as libc::c_int
                                                  + 1 as libc::c_int) as
                                                 usize] as libc::c_int +
                                      1 as libc::c_int) as uint8_t,
                                 0x2000000 as libc::c_int as uint32_t,
                                 0x2000 as libc::c_int as uint32_t);
            I2C_ITConfig(BSTx, 0x2 as libc::c_int as uint32_t, ENABLE);
            dataBufferPointer = 1 as libc::c_int as uint8_t;
            bstMasterWriteTimeout = micros()
        }
    } else if currentTime >
                  bstMasterWriteTimeout.wrapping_add(0x1000 as libc::c_int as
                                                         uint32_t) {
        bstTimeoutUserCallback();
    };
}
/* ************************************************************************************************/
#[no_mangle]
pub unsafe extern "C" fn crc8Cal(mut data_in: uint8_t) {
    /* Polynom = x^8+x^7+x^6+x^4+x^2+1 = x^8+x^7+x^6+x^4+x^2+X^0 */
    let mut Polynom: uint8_t = 0xd5 as libc::c_int as uint8_t;
    let mut MSB_Flag: bool = false;
    /* Step through each bit of the BYTE (8-bits) */
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 8 as libc::c_int {
        /* Clear the Flag */
        MSB_Flag = 0 as libc::c_int != 0;
        /* MSB_Set = 80; */
        if CRC8 as libc::c_int & 0x80 as libc::c_int != 0 {
            MSB_Flag = 1 as libc::c_int != 0
        }
        ::core::ptr::write_volatile(&mut CRC8 as *mut uint8_t,
                                    ((::core::ptr::read_volatile::<uint8_t>(&CRC8
                                                                                as
                                                                                *const uint8_t)
                                          as libc::c_int) << 1 as libc::c_int)
                                        as uint8_t as uint8_t);
        /* MSB_Set = 80; */
        if data_in as libc::c_int & 0x80 as libc::c_int != 0 {
            ::core::ptr::write_volatile(&mut CRC8 as *mut uint8_t,
                                        ::core::ptr::read_volatile::<uint8_t>(&CRC8
                                                                                  as
                                                                                  *const uint8_t).wrapping_add(1))
        }
        data_in = ((data_in as libc::c_int) << 1 as libc::c_int) as uint8_t;
        if MSB_Flag as libc::c_int == 1 as libc::c_int {
            ::core::ptr::write_volatile(&mut CRC8 as *mut uint8_t,
                                        (::core::ptr::read_volatile::<uint8_t>(&CRC8
                                                                                   as
                                                                                   *const uint8_t)
                                             as libc::c_int ^
                                             Polynom as libc::c_int) as
                                            uint8_t as uint8_t)
        }
        i = i.wrapping_add(1)
    };
}
