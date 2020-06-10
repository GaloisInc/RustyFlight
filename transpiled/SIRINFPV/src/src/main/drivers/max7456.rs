use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* Exported constants --------------------------------------------------------*/
    /* * @defgroup DMA_Exported_Constants
  * @{
  */
    /* * @defgroup DMA_data_transfer_direction 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_peripheral_incremented_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_memory_incremented_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_peripheral_data_size 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_memory_data_size 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_circular_normal_mode 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_priority_level 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_memory_to_memory 
  * @{
  */
    /* *
  * @}
  */
    /* * @defgroup DMA_interrupts_definition
  * @{
  */
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
    /* Interrupts and flags management functions **********************************/
    #[no_mangle]
    fn DMA_ITConfig(DMAy_Channelx: *mut DMA_Channel_TypeDef, DMA_IT: uint32_t,
                    NewState: FunctionalState);
    /* DMA transfers management functions *****************************************/
    #[no_mangle]
    fn SPI_I2S_DMACmd(SPIx: *mut SPI_TypeDef, SPI_I2S_DMAReq: uint16_t,
                      NewState: FunctionalState);
    #[no_mangle]
    fn SPI_I2S_GetFlagStatus(SPIx: *mut SPI_TypeDef, SPI_I2S_FLAG: uint16_t)
     -> FlagStatus;
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
    static mut debug: [int16_t; 4];
    #[no_mangle]
    static mut debugMode: uint8_t;
    #[no_mangle]
    fn spiSetDivisor(instance: *mut SPI_TypeDef, divisor: uint16_t);
    #[no_mangle]
    fn spiTransferByte(instance: *mut SPI_TypeDef, data: uint8_t) -> uint8_t;
    #[no_mangle]
    fn spiBusSetInstance(bus: *mut busDevice_t, instance: *mut SPI_TypeDef);
    #[no_mangle]
    fn spiInstanceByDevice(device: SPIDevice) -> *mut SPI_TypeDef;
    #[no_mangle]
    fn dmaSetHandler(identifier: dmaIdentifier_e,
                     callback: dmaCallbackHandlerFuncPtr, priority: uint32_t,
                     userParam: uint32_t);
    #[no_mangle]
    fn IOHi(io: IO_t);
    #[no_mangle]
    fn IOLo(io: IO_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOIsFreeOrPreinit(io: IO_t) -> bool;
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn ledToggle(led: libc::c_int);
    #[no_mangle]
    fn millis() -> timeMs_t;
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __int16_t = libc::c_short;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type int16_t = __int16_t;
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
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
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
  * @brief Serial Peripheral Interface
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct SPI_TypeDef {
    pub CR1: uint16_t,
    pub RESERVED0: uint16_t,
    pub CR2: uint16_t,
    pub RESERVED1: uint16_t,
    pub SR: uint16_t,
    pub RESERVED2: uint16_t,
    pub DR: uint16_t,
    pub RESERVED3: uint16_t,
    pub CRCPR: uint16_t,
    pub RESERVED4: uint16_t,
    pub RXCRCR: uint16_t,
    pub RESERVED5: uint16_t,
    pub TXCRCR: uint16_t,
    pub RESERVED6: uint16_t,
    pub I2SCFGR: uint16_t,
    pub RESERVED7: uint16_t,
    pub I2SPR: uint16_t,
    pub RESERVED8: uint16_t,
}
/* *
  ******************************************************************************
  * @file    stm32f30x_dma.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the DMA firmware
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
/* * @addtogroup DMA
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  DMA Init structures definition
  */
#[derive(Copy, Clone)]
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
pub type C2RustUnnamed_3 = libc::c_uint;
pub const DEBUG_COUNT: C2RustUnnamed_3 = 44;
pub const DEBUG_ANTI_GRAVITY: C2RustUnnamed_3 = 43;
pub const DEBUG_RC_SMOOTHING_RATE: C2RustUnnamed_3 = 42;
pub const DEBUG_RX_SIGNAL_LOSS: C2RustUnnamed_3 = 41;
pub const DEBUG_RC_SMOOTHING: C2RustUnnamed_3 = 40;
pub const DEBUG_ACRO_TRAINER: C2RustUnnamed_3 = 39;
pub const DEBUG_ITERM_RELAX: C2RustUnnamed_3 = 38;
pub const DEBUG_RTH: C2RustUnnamed_3 = 37;
pub const DEBUG_SMARTAUDIO: C2RustUnnamed_3 = 36;
pub const DEBUG_USB: C2RustUnnamed_3 = 35;
pub const DEBUG_CURRENT: C2RustUnnamed_3 = 34;
pub const DEBUG_SDIO: C2RustUnnamed_3 = 33;
pub const DEBUG_RUNAWAY_TAKEOFF: C2RustUnnamed_3 = 32;
pub const DEBUG_CORE_TEMP: C2RustUnnamed_3 = 31;
pub const DEBUG_LIDAR_TF: C2RustUnnamed_3 = 30;
pub const DEBUG_RANGEFINDER_QUALITY: C2RustUnnamed_3 = 29;
pub const DEBUG_RANGEFINDER: C2RustUnnamed_3 = 28;
pub const DEBUG_FPORT: C2RustUnnamed_3 = 27;
pub const DEBUG_SBUS: C2RustUnnamed_3 = 26;
pub const DEBUG_MAX7456_SPICLOCK: C2RustUnnamed_3 = 25;
pub const DEBUG_MAX7456_SIGNAL: C2RustUnnamed_3 = 24;
pub const DEBUG_DUAL_GYRO_DIFF: C2RustUnnamed_3 = 23;
pub const DEBUG_DUAL_GYRO_COMBINE: C2RustUnnamed_3 = 22;
pub const DEBUG_DUAL_GYRO_RAW: C2RustUnnamed_3 = 21;
pub const DEBUG_DUAL_GYRO: C2RustUnnamed_3 = 20;
pub const DEBUG_GYRO_RAW: C2RustUnnamed_3 = 19;
pub const DEBUG_RX_FRSKY_SPI: C2RustUnnamed_3 = 18;
pub const DEBUG_FFT_FREQ: C2RustUnnamed_3 = 17;
pub const DEBUG_FFT_TIME: C2RustUnnamed_3 = 16;
pub const DEBUG_FFT: C2RustUnnamed_3 = 15;
pub const DEBUG_ALTITUDE: C2RustUnnamed_3 = 14;
pub const DEBUG_ESC_SENSOR_TMP: C2RustUnnamed_3 = 13;
pub const DEBUG_ESC_SENSOR_RPM: C2RustUnnamed_3 = 12;
pub const DEBUG_STACK: C2RustUnnamed_3 = 11;
pub const DEBUG_SCHEDULER: C2RustUnnamed_3 = 10;
pub const DEBUG_ESC_SENSOR: C2RustUnnamed_3 = 9;
pub const DEBUG_ANGLERATE: C2RustUnnamed_3 = 8;
pub const DEBUG_RC_INTERPOLATION: C2RustUnnamed_3 = 7;
pub const DEBUG_GYRO_SCALED: C2RustUnnamed_3 = 6;
pub const DEBUG_PIDLOOP: C2RustUnnamed_3 = 5;
pub const DEBUG_ACCELEROMETER: C2RustUnnamed_3 = 4;
pub const DEBUG_GYRO_FILTERED: C2RustUnnamed_3 = 3;
pub const DEBUG_BATTERY: C2RustUnnamed_3 = 2;
pub const DEBUG_CYCLETIME: C2RustUnnamed_3 = 1;
pub const DEBUG_NONE: C2RustUnnamed_3 = 0;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct max7456Config_s {
    pub clockConfig: uint8_t,
    pub csTag: ioTag_t,
    pub spiDevice: uint8_t,
}
pub type max7456Config_t = max7456Config_s;
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
// Video Character Display parameters
pub type VIDEO_SYSTEMS = libc::c_uint;
pub const VIDEO_SYSTEM_NTSC: VIDEO_SYSTEMS = 2;
pub const VIDEO_SYSTEM_PAL: VIDEO_SYSTEMS = 1;
pub const VIDEO_SYSTEM_AUTO: VIDEO_SYSTEMS = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct vcdProfile_s {
    pub video_system: uint8_t,
    pub h_offset: int8_t,
    pub v_offset: int8_t,
}
pub type vcdProfile_t = vcdProfile_s;
pub type I2CDevice = libc::c_int;
pub const I2CDEV_4: I2CDevice = 3;
pub const I2CDEV_3: I2CDevice = 2;
pub const I2CDEV_2: I2CDevice = 1;
pub const I2CDEV_1: I2CDevice = 0;
pub const I2CINVALID: I2CDevice = -1;
pub type busType_e = libc::c_uint;
pub const BUSTYPE_MPU_SLAVE: busType_e = 3;
pub const BUSTYPE_SPI: busType_e = 2;
pub const BUSTYPE_I2C: busType_e = 1;
pub const BUSTYPE_NONE: busType_e = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct busDevice_s {
    pub bustype: busType_e,
    pub busdev_u: C2RustUnnamed_4,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub union C2RustUnnamed_4 {
    pub spi: deviceSpi_s,
    pub i2c: deviceI2C_s,
    pub mpuSlave: deviceMpuSlave_s,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceMpuSlave_s {
    pub master: *const busDevice_s,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceI2C_s {
    pub device: I2CDevice,
    pub address: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct deviceSpi_s {
    pub instance: *mut SPI_TypeDef,
    pub csnPin: IO_t,
}
pub type busDevice_t = busDevice_s;
pub type C2RustUnnamed_5 = libc::c_uint;
pub const SPI_CLOCK_ULTRAFAST: C2RustUnnamed_5 = 2;
pub const SPI_CLOCK_FAST: C2RustUnnamed_5 = 2;
pub const SPI_CLOCK_STANDARD: C2RustUnnamed_5 = 4;
pub const SPI_CLOCK_SLOW: C2RustUnnamed_5 = 128;
pub const SPI_CLOCK_INITIALIZATON: C2RustUnnamed_5 = 256;
pub type SPIDevice = libc::c_int;
pub const SPIDEV_4: SPIDevice = 3;
pub const SPIDEV_3: SPIDevice = 2;
pub const SPIDEV_2: SPIDevice = 1;
pub const SPIDEV_1: SPIDevice = 0;
pub const SPIINVALID: SPIDevice = -1;
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
// millisecond time
pub type timeMs_t = uint32_t;
#[no_mangle]
pub static mut max7456BusDevice: busDevice_t =
    busDevice_t{bustype: BUSTYPE_NONE,
                busdev_u:
                    C2RustUnnamed_4{spi:
                                        deviceSpi_s{instance:
                                                        0 as
                                                            *const SPI_TypeDef
                                                            as
                                                            *mut SPI_TypeDef,
                                                    csnPin:
                                                        0 as
                                                            *const libc::c_void
                                                            as
                                                            *mut libc::c_void,},},};
#[no_mangle]
pub static mut busdev: *mut busDevice_t =
    unsafe { &max7456BusDevice as *const busDevice_t as *mut busDevice_t };
static mut max7456SpiClock: uint16_t =
    SPI_CLOCK_STANDARD as libc::c_int as uint16_t;
#[no_mangle]
pub static mut maxScreenSize: uint16_t = 480 as libc::c_int as uint16_t;
// We write everything in screenBuffer and then compare
// screenBuffer with shadowBuffer to upgrade only changed chars.
// This solution is faster then redrawing entire screen.
static mut screenBuffer: [uint8_t; 520] = [0; 520];
// For faster writes we use memcpy so we need some space to don't overwrite buffer
static mut shadowBuffer: [uint8_t; 480] = [0; 480];
#[no_mangle]
pub static mut dmaTransactionInProgress: bool = 0 as libc::c_int != 0;
static mut spiBuff: [uint8_t; 600] = [0; 600];
static mut videoSignalCfg: uint8_t = 0;
static mut videoSignalReg: uint8_t = 0x8 as libc::c_int as uint8_t;
// OSD_ENABLE required to trigger first ReInit
static mut displayMemoryModeReg: uint8_t = 0 as libc::c_int as uint8_t;
static mut hosRegValue: uint8_t = 0;
// HOS (Horizontal offset register) value
static mut vosRegValue: uint8_t = 0;
// VOS (Vertical offset register) value
static mut max7456Lock: bool = 0 as libc::c_int != 0;
static mut fontIsLoading: bool = 0 as libc::c_int != 0;
static mut max7456DeviceType: uint8_t = 0;
unsafe extern "C" fn max7456Send(mut add: uint8_t, mut data: uint8_t)
 -> uint8_t {
    spiTransferByte((*busdev).busdev_u.spi.instance,
                    add); // Wait for prev DMA transaction
    return spiTransferByte((*busdev).busdev_u.spi.instance, data);
}
unsafe extern "C" fn max7456SendDma(mut tx_buffer: *mut libc::c_void,
                                    mut rx_buffer: *mut libc::c_void,
                                    mut buffer_size: uint16_t) {
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
    static mut dummy: [uint16_t; 1] = [0xffff as libc::c_int as uint16_t];
    while dmaTransactionInProgress { }
    DMA_DeInit((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x41c
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_Channel_TypeDef);
    DMA_DeInit((0x40000000 as libc::c_int as
                    uint32_t).wrapping_add(0x20000 as libc::c_int as
                                               libc::c_uint).wrapping_add(0x408
                                                                              as
                                                                              libc::c_int
                                                                              as
                                                                              libc::c_uint)
                   as *mut DMA_Channel_TypeDef);
    // Common to both channels
    DMA_StructInit(&mut DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr =
        &mut (*(*busdev).busdev_u.spi.instance).DR as *mut uint16_t as
            uint32_t;
    DMA_InitStructure.DMA_PeripheralDataSize = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryDataSize = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_PeripheralInc = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_BufferSize = buffer_size;
    DMA_InitStructure.DMA_Mode = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_Priority = 0 as libc::c_int as uint32_t;
    // Rx Channel
    DMA_InitStructure.DMA_MemoryBaseAddr =
        if !rx_buffer.is_null() {
            rx_buffer as uint32_t
        } else { dummy.as_mut_ptr() as uint32_t };
    DMA_InitStructure.DMA_DIR = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryInc =
        if !rx_buffer.is_null() {
            0x80 as libc::c_int as uint32_t
        } else { 0 as libc::c_int as uint32_t };
    DMA_Init((0x40000000 as libc::c_int as
                  uint32_t).wrapping_add(0x20000 as libc::c_int as
                                             libc::c_uint).wrapping_add(0x408
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                 as *mut DMA_Channel_TypeDef, &mut DMA_InitStructure);
    DMA_Cmd((0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x20000 as libc::c_int as
                                            libc::c_uint).wrapping_add(0x408
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint)
                as *mut DMA_Channel_TypeDef, ENABLE);
    // Tx channel
    DMA_InitStructure.DMA_MemoryBaseAddr =
        tx_buffer as uint32_t; //max7456_screen;
    DMA_InitStructure.DMA_DIR = 0x10 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryInc = 0x80 as libc::c_int as uint32_t;
    DMA_Init((0x40000000 as libc::c_int as
                  uint32_t).wrapping_add(0x20000 as libc::c_int as
                                             libc::c_uint).wrapping_add(0x41c
                                                                            as
                                                                            libc::c_int
                                                                            as
                                                                            libc::c_uint)
                 as *mut DMA_Channel_TypeDef, &mut DMA_InitStructure);
    DMA_Cmd((0x40000000 as libc::c_int as
                 uint32_t).wrapping_add(0x20000 as libc::c_int as
                                            libc::c_uint).wrapping_add(0x41c
                                                                           as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_uint)
                as *mut DMA_Channel_TypeDef, ENABLE);
    DMA_ITConfig((0x40000000 as libc::c_int as
                      uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                 libc::c_uint).wrapping_add(0x408
                                                                                as
                                                                                libc::c_int
                                                                                as
                                                                                libc::c_uint)
                     as *mut DMA_Channel_TypeDef,
                 0x2 as libc::c_int as uint32_t, ENABLE);
    // Enable SPI TX/RX request
    IOLo((*busdev).busdev_u.spi.csnPin);
    ::core::ptr::write_volatile(&mut dmaTransactionInProgress as *mut bool,
                                1 as libc::c_int != 0);
    SPI_I2S_DMACmd((*busdev).busdev_u.spi.instance,
                   (0x1 as libc::c_int as uint16_t as libc::c_int |
                        0x2 as libc::c_int as uint16_t as libc::c_int) as
                       uint16_t, ENABLE);
}
#[no_mangle]
pub unsafe extern "C" fn max7456_dma_irq_handler(mut descriptor:
                                                     *mut dmaChannelDescriptor_t) {
    if (*(*descriptor).dma).ISR &
           (0x2 as libc::c_int as uint32_t) <<
               (*descriptor).flagsShift as libc::c_int != 0 {
        DMA_Cmd((0x40000000 as libc::c_int as
                     uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                libc::c_uint).wrapping_add(0x408
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint)
                    as *mut DMA_Channel_TypeDef, DISABLE);
        // Make sure SPI DMA transfer is complete
        while SPI_I2S_GetFlagStatus((*busdev).busdev_u.spi.instance,
                                    0x2 as libc::c_int as uint16_t) as
                  libc::c_uint == RESET as libc::c_int as libc::c_uint {
        }
        while SPI_I2S_GetFlagStatus((*busdev).busdev_u.spi.instance,
                                    0x80 as libc::c_int as uint16_t) as
                  libc::c_uint == SET as libc::c_int as libc::c_uint {
        }
        // Empty RX buffer. RX DMA takes care of it if enabled.
        // This should be done after transmission finish!!!
        while SPI_I2S_GetFlagStatus((*busdev).busdev_u.spi.instance,
                                    0x1 as libc::c_int as uint16_t) as
                  libc::c_uint == SET as libc::c_int as libc::c_uint {
        }
        DMA_Cmd((0x40000000 as libc::c_int as
                     uint32_t).wrapping_add(0x20000 as libc::c_int as
                                                libc::c_uint).wrapping_add(0x41c
                                                                               as
                                                                               libc::c_int
                                                                               as
                                                                               libc::c_uint)
                    as *mut DMA_Channel_TypeDef, DISABLE);
        ::core::ptr::write_volatile(&mut (*(*descriptor).dma).IFCR as
                                        *mut uint32_t,
                                    (0x2 as libc::c_int as uint32_t) <<
                                        (*descriptor).flagsShift as
                                            libc::c_int);
        SPI_I2S_DMACmd((*busdev).busdev_u.spi.instance,
                       (0x1 as libc::c_int as uint16_t as libc::c_int |
                            0x2 as libc::c_int as uint16_t as libc::c_int) as
                           uint16_t, DISABLE);
        IOHi((*busdev).busdev_u.spi.csnPin);
        ::core::ptr::write_volatile(&mut dmaTransactionInProgress as
                                        *mut bool, 0 as libc::c_int != 0)
    }
    if (*(*descriptor).dma).ISR &
           (0x4 as libc::c_int as uint32_t) <<
               (*descriptor).flagsShift as libc::c_int != 0 {
        ::core::ptr::write_volatile(&mut (*(*descriptor).dma).IFCR as
                                        *mut uint32_t,
                                    (0x4 as libc::c_int as uint32_t) <<
                                        (*descriptor).flagsShift as
                                            libc::c_int)
    }
    if (*(*descriptor).dma).ISR &
           (0x8 as libc::c_int as uint32_t) <<
               (*descriptor).flagsShift as libc::c_int != 0 {
        ::core::ptr::write_volatile(&mut (*(*descriptor).dma).IFCR as
                                        *mut uint32_t,
                                    (0x8 as libc::c_int as uint32_t) <<
                                        (*descriptor).flagsShift as
                                            libc::c_int)
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456GetRowsCount() -> uint8_t {
    return if videoSignalReg as libc::c_int & 0x40 as libc::c_int != 0 {
               16 as libc::c_int
           } else { 13 as libc::c_int } as uint8_t;
}
#[no_mangle]
pub unsafe extern "C" fn max7456ReInit() {
    let mut srdata: uint8_t = 0 as libc::c_int as uint8_t;
    static mut firstInit: bool = 1 as libc::c_int != 0;
    IOLo((*busdev).busdev_u.spi.csnPin);
    match videoSignalCfg as libc::c_int {
        1 => {
            videoSignalReg =
                (0x40 as libc::c_int | 0x8 as libc::c_int) as uint8_t
        }
        2 => {
            videoSignalReg =
                (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
        }
        0 => {
            srdata =
                max7456Send(0xa0 as libc::c_int as uint8_t,
                            0 as libc::c_int as uint8_t);
            if srdata as libc::c_int & 0x4 as libc::c_int == 0 &&
                   srdata as libc::c_int & 0x2 as libc::c_int != 0 {
                videoSignalReg =
                    (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            } else if srdata as libc::c_int & 0x4 as libc::c_int == 0 &&
                          srdata as libc::c_int & 0x1 as libc::c_int != 0 {
                videoSignalReg =
                    (0x40 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            } else {
                // No valid input signal, fallback to default (XXX NTSC for now)
                videoSignalReg =
                    (0 as libc::c_int | 0x8 as libc::c_int) as uint8_t
            }
        }
        _ => { }
    } // NTSC
    if videoSignalReg as libc::c_int & 0x40 as libc::c_int != 0 {
        //PAL
        maxScreenSize = 480 as libc::c_int as uint16_t
    } else { maxScreenSize = 390 as libc::c_int as uint16_t }
    // Set all rows to same charactor black/white level
    max7456Brightness(0 as libc::c_int as uint8_t,
                      2 as libc::c_int as uint8_t);
    // Re-enable MAX7456 (last function call disables it)
    IOLo((*busdev).busdev_u.spi.csnPin);
    // Make sure the Max7456 is enabled
    max7456Send(0 as libc::c_int as uint8_t, videoSignalReg);
    max7456Send(0x2 as libc::c_int as uint8_t, hosRegValue);
    max7456Send(0x3 as libc::c_int as uint8_t, vosRegValue);
    max7456Send(0x4 as libc::c_int as uint8_t,
                (displayMemoryModeReg as libc::c_int | 0x4 as libc::c_int) as
                    uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    // Clear shadow to force redraw all screen in non-dma mode.
    memset(shadowBuffer.as_mut_ptr() as *mut libc::c_void, 0 as libc::c_int,
           maxScreenSize as libc::c_ulong);
    if firstInit {
        max7456DrawScreenSlow();
        firstInit = 0 as libc::c_int != 0
    };
}
// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)
#[no_mangle]
pub unsafe extern "C" fn max7456Init(mut max7456Config:
                                         *const max7456Config_t,
                                     mut pVcdProfile: *const vcdProfile_t,
                                     mut cpuOverclock: bool) -> bool {
    max7456HardwareReset();
    if (*max7456Config).csTag == 0 { return 0 as libc::c_int != 0 }
    (*busdev).busdev_u.spi.csnPin = IOGetByTag((*max7456Config).csTag);
    if !IOIsFreeOrPreinit((*busdev).busdev_u.spi.csnPin) {
        return 0 as libc::c_int != 0
    }
    IOInit((*busdev).busdev_u.spi.csnPin, OWNER_OSD_CS,
           0 as libc::c_int as uint8_t);
    IOConfigGPIO((*busdev).busdev_u.spi.csnPin,
                 (GPIO_Mode_OUT as libc::c_int |
                      (GPIO_Speed_Level_3 as libc::c_int) << 2 as libc::c_int
                      | (GPIO_OType_PP as libc::c_int) << 4 as libc::c_int |
                      (GPIO_PuPd_NOPULL as libc::c_int) << 5 as libc::c_int)
                     as ioConfig_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    spiBusSetInstance(busdev,
                      spiInstanceByDevice(((*max7456Config).spiDevice as
                                               libc::c_int - 1 as libc::c_int)
                                              as SPIDevice));
    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // Do this at half the speed for safety.
    spiSetDivisor((*busdev).busdev_u.spi.instance,
                  (SPI_CLOCK_STANDARD as libc::c_int * 2 as libc::c_int) as
                      uint16_t); // CA[8] bit
    max7456Send(0xa as libc::c_int as uint8_t,
                ((1 as libc::c_int) << 6 as libc::c_int) as uint8_t);
    if max7456Send((0xa as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                   0xff as libc::c_int as uint8_t) as libc::c_int &
           (1 as libc::c_int) << 6 as libc::c_int != 0 {
        max7456DeviceType = 1 as libc::c_int as uint8_t
    } else { max7456DeviceType = 0 as libc::c_int as uint8_t }
    spiSetDivisor((*busdev).busdev_u.spi.instance, max7456SpiClock);
    // force soft reset on Max7456
    IOLo((*busdev).busdev_u.spi.csnPin);
    max7456Send(0 as libc::c_int as uint8_t, 0x2 as libc::c_int as uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    // Setup values to write to registers
    videoSignalCfg = (*pVcdProfile).video_system;
    hosRegValue =
        (32 as libc::c_int - (*pVcdProfile).h_offset as libc::c_int) as
            uint8_t;
    vosRegValue =
        (16 as libc::c_int - (*pVcdProfile).v_offset as libc::c_int) as
            uint8_t;
    dmaSetHandler(DMA2_CH1_HANDLER,
                  Some(max7456_dma_irq_handler as
                           unsafe extern "C" fn(_:
                                                    *mut dmaChannelDescriptor_t)
                               -> ()),
                  (((3 as libc::c_int) <<
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
                        0 as libc::c_int &
                            0xf as libc::c_int >>
                                (7 as libc::c_int as
                                     libc::c_uint).wrapping_sub(0x500 as
                                                                    libc::c_int
                                                                    as
                                                                    uint32_t
                                                                    >>
                                                                    8 as
                                                                        libc::c_int))
                       << 4 as libc::c_int & 0xf0 as libc::c_int) as uint32_t,
                  0 as libc::c_int as uint32_t);
    // Real init will be made later when driver detect idle.
    return 1 as libc::c_int != 0;
}
/* *
 * Sets inversion of black and white pixels.
 */
#[no_mangle]
pub unsafe extern "C" fn max7456Invert(mut invert: bool) {
    if invert {
        displayMemoryModeReg =
            (displayMemoryModeReg as libc::c_int | 0x8 as libc::c_int) as
                uint8_t
    } else {
        displayMemoryModeReg =
            (displayMemoryModeReg as libc::c_int & !(0x8 as libc::c_int)) as
                uint8_t
    }
    IOLo((*busdev).busdev_u.spi.csnPin);
    max7456Send(0x4 as libc::c_int as uint8_t, displayMemoryModeReg);
    IOHi((*busdev).busdev_u.spi.csnPin);
}
/* *
 * Sets the brighness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
#[no_mangle]
pub unsafe extern "C" fn max7456Brightness(mut black: uint8_t,
                                           mut white: uint8_t) {
    let reg: uint8_t =
        ((black as libc::c_int) << 2 as libc::c_int |
             3 as libc::c_int - white as libc::c_int) as uint8_t;
    IOLo((*busdev).busdev_u.spi.csnPin);
    let mut i: libc::c_int = 0x10 as libc::c_int;
    while i <= 0x1f as libc::c_int { max7456Send(i as uint8_t, reg); i += 1 }
    IOHi((*busdev).busdev_u.spi.csnPin);
}
//just fill with spaces with some tricks
#[no_mangle]
pub unsafe extern "C" fn max7456ClearScreen() {
    memset(screenBuffer.as_mut_ptr() as *mut libc::c_void,
           0x20 as libc::c_int, 480 as libc::c_int as libc::c_ulong);
}
#[no_mangle]
pub unsafe extern "C" fn max7456GetScreenBuffer() -> *mut uint8_t {
    return screenBuffer.as_mut_ptr();
}
#[no_mangle]
pub unsafe extern "C" fn max7456WriteChar(mut x: uint8_t, mut y: uint8_t,
                                          mut c: uint8_t) {
    screenBuffer[(y as libc::c_int * 30 as libc::c_int + x as libc::c_int) as
                     usize] = c;
}
#[no_mangle]
pub unsafe extern "C" fn max7456Write(mut x: uint8_t, mut y: uint8_t,
                                      mut buff: *const libc::c_char) {
    let mut i: libc::c_int = 0 as libc::c_int;
    while *buff.offset(i as isize) != 0 {
        if x as libc::c_int + i < 30 as libc::c_int {
            // Do not write over screen
            screenBuffer[(y as libc::c_int * 30 as libc::c_int +
                              x as libc::c_int + i) as usize] =
                *buff.offset(i as isize) as uint8_t
        }
        i += 1
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456DmaInProgress() -> bool {
    return dmaTransactionInProgress;
}
#[no_mangle]
pub unsafe extern "C" fn max7456BuffersSynced() -> bool {
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < maxScreenSize as libc::c_int {
        if screenBuffer[i as usize] as libc::c_int !=
               shadowBuffer[i as usize] as libc::c_int {
            return 0 as libc::c_int != 0
        }
        i += 1
    }
    return 1 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456ReInitIfRequired() {
    static mut lastSigCheckMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut videoDetectTimeMs: uint32_t = 0 as libc::c_int as uint32_t;
    static mut reInitCount: uint16_t = 0 as libc::c_int as uint16_t;
    IOLo((*busdev).busdev_u.spi.csnPin);
    let stallCheck: uint8_t =
        max7456Send((0 as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                    0 as libc::c_int as uint8_t);
    IOHi((*busdev).busdev_u.spi.csnPin);
    let nowMs: timeMs_t = millis();
    if stallCheck as libc::c_int != videoSignalReg as libc::c_int {
        max7456ReInit();
    } else if videoSignalCfg as libc::c_int ==
                  VIDEO_SYSTEM_AUTO as libc::c_int &&
                  nowMs.wrapping_sub(lastSigCheckMs) >
                      1000 as libc::c_int as libc::c_uint {
        // Adjust output format based on the current input format.
        IOLo((*busdev).busdev_u.spi.csnPin);
        let videoSense: uint8_t =
            max7456Send(0xa0 as libc::c_int as uint8_t,
                        0 as libc::c_int as uint8_t);
        IOHi((*busdev).busdev_u.spi.csnPin);
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[0 as libc::c_int as usize] =
                (videoSignalReg as libc::c_int & 0x40 as libc::c_int) as
                    int16_t
        }
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[1 as libc::c_int as usize] =
                (videoSense as libc::c_int & 0x7 as libc::c_int) as int16_t
        }
        if debugMode as libc::c_int == DEBUG_MAX7456_SIGNAL as libc::c_int {
            debug[3 as libc::c_int as usize] =
                max7456GetRowsCount() as int16_t
        }
        if videoSense as libc::c_int & 0x4 as libc::c_int != 0 {
            videoDetectTimeMs = 0 as libc::c_int as uint32_t
        } else if videoSense as libc::c_int & 0x4 as libc::c_int == 0 &&
                      videoSense as libc::c_int & 0x1 as libc::c_int != 0 &&
                      videoSignalReg as libc::c_int & 0x40 as libc::c_int ==
                          0 as libc::c_int ||
                      videoSense as libc::c_int & 0x4 as libc::c_int == 0 &&
                          videoSense as libc::c_int & 0x1 as libc::c_int == 0
                          &&
                          videoSignalReg as libc::c_int & 0x40 as libc::c_int
                              == 0x40 as libc::c_int {
            if videoDetectTimeMs != 0 {
                if millis().wrapping_sub(videoDetectTimeMs) >
                       100 as libc::c_int as libc::c_uint {
                    max7456ReInit();
                    if debugMode as libc::c_int ==
                           DEBUG_MAX7456_SIGNAL as libc::c_int {
                        reInitCount = reInitCount.wrapping_add(1);
                        debug[2 as libc::c_int as usize] =
                            reInitCount as int16_t
                    }
                }
            } else {
                // Wait for signal to stabilize
                videoDetectTimeMs = millis()
            }
        }
        lastSigCheckMs = nowMs
    };
    //------------   end of (re)init-------------------------------------
}
#[no_mangle]
pub unsafe extern "C" fn max7456DrawScreen() {
    static mut pos: uint16_t = 0 as libc::c_int as uint16_t;
    if !max7456Lock && !fontIsLoading {
        // (Re)Initialize MAX7456 at startup or stall is detected.
        max7456Lock = 1 as libc::c_int != 0;
        max7456ReInitIfRequired();
        let mut buff_len: libc::c_int = 0 as libc::c_int;
        let mut k: libc::c_int = 0 as libc::c_int;
        while k < 100 as libc::c_int {
            if screenBuffer[pos as usize] as libc::c_int !=
                   shadowBuffer[pos as usize] as libc::c_int {
                let fresh0 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh0 as usize] = 0x5 as libc::c_int as uint8_t;
                let fresh1 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh1 as usize] =
                    (pos as libc::c_int >> 8 as libc::c_int) as uint8_t;
                let fresh2 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh2 as usize] = 0x6 as libc::c_int as uint8_t;
                let fresh3 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh3 as usize] =
                    (pos as libc::c_int & 0xff as libc::c_int) as uint8_t;
                let fresh4 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh4 as usize] = 0x7 as libc::c_int as uint8_t;
                let fresh5 = buff_len;
                buff_len = buff_len + 1;
                spiBuff[fresh5 as usize] = screenBuffer[pos as usize];
                shadowBuffer[pos as usize] = screenBuffer[pos as usize]
            }
            pos = pos.wrapping_add(1);
            if pos as libc::c_int >= maxScreenSize as libc::c_int {
                pos = 0 as libc::c_int as uint16_t;
                break ;
            } else { k += 1 }
        }
        if buff_len != 0 {
            max7456SendDma(spiBuff.as_mut_ptr() as *mut libc::c_void,
                           0 as *mut libc::c_void, buff_len as uint16_t);
            // MAX7456_DMA_CHANNEL_TX
        }
        max7456Lock = 0 as libc::c_int != 0
    };
}
unsafe extern "C" fn max7456DrawScreenSlow() {
    let mut escapeCharFound: bool = false;
    IOLo((*busdev).busdev_u.spi.csnPin);
    // Enable auto-increment mode and update every character in the screenBuffer.
    // The "escape" character 0xFF must be skipped as it causes the MAX7456 to exit auto-increment mode.
    max7456Send(0x5 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    max7456Send(0x6 as libc::c_int as uint8_t, 0 as libc::c_int as uint8_t);
    max7456Send(0x4 as libc::c_int as uint8_t,
                (displayMemoryModeReg as libc::c_int | 1 as libc::c_int) as
                    uint8_t);
    let mut xx: libc::c_int = 0 as libc::c_int;
    while xx < maxScreenSize as libc::c_int {
        if screenBuffer[xx as usize] as libc::c_int == 0xff as libc::c_int {
            escapeCharFound = 1 as libc::c_int != 0;
            max7456Send(0x7 as libc::c_int as uint8_t, ' ' as i32 as uint8_t);
            // replace the 0xFF character with a blank in the first pass to avoid terminating auto-increment
        } else {
            max7456Send(0x7 as libc::c_int as uint8_t,
                        screenBuffer[xx as usize]);
        }
        shadowBuffer[xx as usize] = screenBuffer[xx as usize];
        xx += 1
    }
    max7456Send(0x7 as libc::c_int as uint8_t,
                0xff as libc::c_int as uint8_t);
    max7456Send(0x4 as libc::c_int as uint8_t, displayMemoryModeReg);
    // If we found any of the "escape" character 0xFF, then make a second pass
    // to update them with direct addressing
    if escapeCharFound {
        let mut xx_0: libc::c_int = 0 as libc::c_int;
        while xx_0 < maxScreenSize as libc::c_int {
            if screenBuffer[xx_0 as usize] as libc::c_int ==
                   0xff as libc::c_int {
                max7456Send(0x5 as libc::c_int as uint8_t,
                            (xx_0 >> 8 as libc::c_int) as uint8_t);
                max7456Send(0x6 as libc::c_int as uint8_t,
                            (xx_0 & 0xff as libc::c_int) as uint8_t);
                max7456Send(0x7 as libc::c_int as uint8_t,
                            0xff as libc::c_int as uint8_t);
            }
            xx_0 += 1
        }
    }
    IOHi((*busdev).busdev_u.spi.csnPin);
}
// should not be used when armed
#[no_mangle]
pub unsafe extern "C" fn max7456RefreshAll() {
    if !max7456Lock {
        while dmaTransactionInProgress { }
        max7456Lock = 1 as libc::c_int != 0;
        max7456ReInitIfRequired();
        max7456DrawScreenSlow();
        max7456Lock = 0 as libc::c_int != 0
    };
}
#[no_mangle]
pub unsafe extern "C" fn max7456WriteNvm(mut char_address: uint8_t,
                                         mut font_data: *const uint8_t) {
    while dmaTransactionInProgress { }
    while max7456Lock { }
    max7456Lock = 1 as libc::c_int != 0;
    IOLo((*busdev).busdev_u.spi.csnPin);
    // disable display
    fontIsLoading = 1 as libc::c_int != 0; // set start address high
    max7456Send(0 as libc::c_int as uint8_t,
                0 as libc::c_int as uint8_t); //set start address low
    max7456Send(0x9 as libc::c_int as uint8_t, char_address);
    let mut x: libc::c_int = 0 as libc::c_int;
    while x < 54 as libc::c_int {
        max7456Send(0xa as libc::c_int as uint8_t, x as uint8_t);
        max7456Send(0xb as libc::c_int as uint8_t,
                    *font_data.offset(x as isize));
        ledToggle(0 as libc::c_int);
        x += 1
    }
    // Transfer 54 bytes from shadow ram to NVM
    max7456Send(0x8 as libc::c_int as uint8_t,
                0xa0 as libc::c_int as uint8_t);
    // Wait until bit 5 in the status register returns to 0 (12ms)
    while max7456Send(0xa0 as libc::c_int as uint8_t,
                      0 as libc::c_int as uint8_t) as libc::c_int &
              0x20 as libc::c_int != 0 as libc::c_int {
    }
    IOHi((*busdev).busdev_u.spi.csnPin);
    max7456Lock = 0 as libc::c_int != 0;
}
#[no_mangle]
pub unsafe extern "C" fn max7456HardwareReset() { }
// USE_MAX7456
