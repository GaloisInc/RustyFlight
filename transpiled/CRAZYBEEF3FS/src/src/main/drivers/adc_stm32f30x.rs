use ::libc;
extern "C" {
    #[no_mangle]
    fn memset(_: *mut libc::c_void, _: libc::c_int, _: libc::c_ulong)
     -> *mut libc::c_void;
    /* Initialization and Configuration functions *********************************/
    #[no_mangle]
    fn ADC_Init(ADCx: *mut ADC_TypeDef, ADC_InitStruct: *mut ADC_InitTypeDef);
    #[no_mangle]
    fn ADC_StructInit(ADC_InitStruct: *mut ADC_InitTypeDef);
    #[no_mangle]
    fn ADC_CommonInit(ADCx: *mut ADC_TypeDef,
                      ADC_CommonInitStruct: *mut ADC_CommonInitTypeDef);
    #[no_mangle]
    fn ADC_CommonStructInit(ADC_CommonInitStruct: *mut ADC_CommonInitTypeDef);
    #[no_mangle]
    fn ADC_Cmd(ADCx: *mut ADC_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn ADC_StartCalibration(ADCx: *mut ADC_TypeDef);
    #[no_mangle]
    fn ADC_SelectCalibrationMode(ADCx: *mut ADC_TypeDef,
                                 ADC_CalibrationMode: uint32_t);
    #[no_mangle]
    fn ADC_GetCalibrationStatus(ADCx: *mut ADC_TypeDef) -> FlagStatus;
    #[no_mangle]
    fn ADC_VoltageRegulatorCmd(ADCx: *mut ADC_TypeDef,
                               NewState: FunctionalState);
    /* Channels Configuration functions ***********************************/
    #[no_mangle]
    fn ADC_RegularChannelConfig(ADCx: *mut ADC_TypeDef, ADC_Channel: uint8_t,
                                Rank: uint8_t, ADC_SampleTime: uint8_t);
    #[no_mangle]
    fn ADC_StartConversion(ADCx: *mut ADC_TypeDef);
    /* Regular Channels DMA Configuration functions *******************************/
    #[no_mangle]
    fn ADC_DMACmd(ADCx: *mut ADC_TypeDef, NewState: FunctionalState);
    #[no_mangle]
    fn ADC_DMAConfig(ADCx: *mut ADC_TypeDef, ADC_DMAMode: uint32_t);
    #[no_mangle]
    fn ADC_GetFlagStatus(ADCx: *mut ADC_TypeDef, ADC_FLAG: uint32_t)
     -> FlagStatus;
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
    /* Peripheral clocks configuration functions **********************************/
    #[no_mangle]
    fn RCC_ADCCLKConfig(RCC_PLLCLK: uint32_t);
    #[no_mangle]
    fn IOInit(io: IO_t, owner: resourceOwner_e, index: uint8_t);
    #[no_mangle]
    fn IOGetByTag(tag: ioTag_t) -> IO_t;
    #[no_mangle]
    fn IOConfigGPIO(io: IO_t, cfg: ioConfig_t);
    #[no_mangle]
    fn delay(ms: timeMs_t);
    #[no_mangle]
    fn adcDeviceByInstance(instance: *mut ADC_TypeDef) -> ADCDevice;
    #[no_mangle]
    static mut adcOperatingConfig: [adcOperatingConfig_t; 4];
    #[no_mangle]
    static mut adcValues: [uint16_t; 4];
    #[no_mangle]
    fn adcChannelByTag(ioTag: ioTag_t) -> uint8_t;
    #[no_mangle]
    fn adcVerifyPin(tag: ioTag_t, device: ADCDevice) -> bool;
    #[no_mangle]
    fn RCC_ClockCmd(periphTag: rccPeriphTag_t, NewState: FunctionalState);
    #[no_mangle]
    fn dmaGetIdentifier(channel: *const DMA_Channel_TypeDef)
     -> dmaIdentifier_e;
    #[no_mangle]
    fn dmaInit(identifier: dmaIdentifier_e, owner: resourceOwner_e,
               resourceIndex: uint8_t);
}
pub type __int8_t = libc::c_schar;
pub type __uint8_t = libc::c_uchar;
pub type __uint16_t = libc::c_ushort;
pub type __uint32_t = libc::c_uint;
pub type int8_t = __int8_t;
pub type uint8_t = __uint8_t;
pub type uint16_t = __uint16_t;
pub type uint32_t = __uint32_t;
/* !< Read Only */
pub type FlagStatus = libc::c_uint;
pub const SET: FlagStatus = 1;
pub const RESET: FlagStatus = 0;
pub type FunctionalState = libc::c_uint;
pub const ENABLE: FunctionalState = 1;
pub const DISABLE: FunctionalState = 0;
/* *
  * @}
  */
/* * @addtogroup Peripheral_registers_structures
  * @{
  */
/* * 
  * @brief Analog to Digital Converter  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_TypeDef {
    pub ISR: uint32_t,
    pub IER: uint32_t,
    pub CR: uint32_t,
    pub CFGR: uint32_t,
    pub RESERVED0: uint32_t,
    pub SMPR1: uint32_t,
    pub SMPR2: uint32_t,
    pub RESERVED1: uint32_t,
    pub TR1: uint32_t,
    pub TR2: uint32_t,
    pub TR3: uint32_t,
    pub RESERVED2: uint32_t,
    pub SQR1: uint32_t,
    pub SQR2: uint32_t,
    pub SQR3: uint32_t,
    pub SQR4: uint32_t,
    pub DR: uint32_t,
    pub RESERVED3: uint32_t,
    pub RESERVED4: uint32_t,
    pub JSQR: uint32_t,
    pub RESERVED5: [uint32_t; 4],
    pub OFR1: uint32_t,
    pub OFR2: uint32_t,
    pub OFR3: uint32_t,
    pub OFR4: uint32_t,
    pub RESERVED6: [uint32_t; 4],
    pub JDR1: uint32_t,
    pub JDR2: uint32_t,
    pub JDR3: uint32_t,
    pub JDR4: uint32_t,
    pub RESERVED7: [uint32_t; 4],
    pub AWD2CR: uint32_t,
    pub AWD3CR: uint32_t,
    pub RESERVED8: uint32_t,
    pub RESERVED9: uint32_t,
    pub DIFSEL: uint32_t,
    pub CALFACT: uint32_t,
}
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
  ******************************************************************************
  * @file    stm32f30x_adc.h
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    04-April-2014
  * @brief   This file contains all the functions prototypes for the ADC firmware 
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
/* * @addtogroup ADC
  * @{
  */
/* Exported types ------------------------------------------------------------*/
/* * 
  * @brief  ADC Init structure definition  
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_InitTypeDef {
    pub ADC_ContinuousConvMode: uint32_t,
    pub ADC_Resolution: uint32_t,
    pub ADC_ExternalTrigConvEvent: uint32_t,
    pub ADC_ExternalTrigEventEdge: uint32_t,
    pub ADC_DataAlign: uint32_t,
    pub ADC_OverrunMode: uint32_t,
    pub ADC_AutoInjMode: uint32_t,
    pub ADC_NbrOfRegChannel: uint8_t,
}
/* *
  * @}
  */
#[derive(Copy, Clone)]
#[repr(C)]
pub struct ADC_CommonInitTypeDef {
    pub ADC_Mode: uint32_t,
    pub ADC_Clock: uint32_t,
    pub ADC_DMAAccessMode: uint32_t,
    pub ADC_DMAMode: uint32_t,
    pub ADC_TwoSamplingDelay: uint8_t,
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
/* * @defgroup Configuration_Pull-Up_Pull-Down_enumeration 
  * @{
  */
pub type C2RustUnnamed_1 = libc::c_uint;
pub const GPIO_PuPd_DOWN: C2RustUnnamed_1 = 2;
pub const GPIO_PuPd_UP: C2RustUnnamed_1 = 1;
pub const GPIO_PuPd_NOPULL: C2RustUnnamed_1 = 0;
pub type ioTag_t = uint8_t;
pub type IO_t = *mut libc::c_void;
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
// packet tag to specify IO pin
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
// millisecond time
pub type timeMs_t = uint32_t;
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
pub type ADCDevice = libc::c_int;
pub const ADCDEV_COUNT: ADCDevice = 4;
pub const ADCDEV_4: ADCDevice = 3;
pub const ADCDEV_3: ADCDevice = 2;
pub const ADCDEV_2: ADCDevice = 1;
pub const ADCDEV_1: ADCDevice = 0;
pub const ADCINVALID: ADCDevice = -1;
pub type C2RustUnnamed_2 = libc::c_uint;
pub const ADC_CHANNEL_COUNT: C2RustUnnamed_2 = 4;
pub const ADC_RSSI: C2RustUnnamed_2 = 3;
pub const ADC_EXTERNAL1: C2RustUnnamed_2 = 2;
pub const ADC_CURRENT: C2RustUnnamed_2 = 1;
pub const ADC_BATTERY: C2RustUnnamed_2 = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcOperatingConfig_s {
    pub tag: ioTag_t,
    pub adcChannel: uint8_t,
    pub dmaIndex: uint8_t,
    pub enabled: bool,
    pub sampleTime: uint8_t,
}
pub type adcOperatingConfig_t = adcOperatingConfig_s;
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
pub type adcDevice_t = adcDevice_s;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcDevice_s {
    pub ADCx: *mut ADC_TypeDef,
    pub rccADC: rccPeriphTag_t,
    pub DMAy_Channelx: *mut DMA_Channel_TypeDef,
}
// ADC1_INxx channel number
// index into DMA buffer in case of sparse channels
// ADCDevice
// make sure that default value (0) does not enable anything
pub const RCC_AHB: rcc_reg = 1;
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct adcTagMap_s {
    pub tag: ioTag_t,
    pub devices: uint8_t,
    pub channel: uint8_t,
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
pub type adcTagMap_t = adcTagMap_s;
// F1 pins have uniform connection to ADC instances
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
pub const RCC_EMPTY: rcc_reg = 0;
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
pub static mut adcHardware: [adcDevice_t; 3] =
    [adcDevice_t{ADCx: 0 as *mut ADC_TypeDef,
                 rccADC: 0,
                 DMAy_Channelx: 0 as *mut DMA_Channel_TypeDef,}; 3];
#[no_mangle]
pub static mut adcTagMap: [adcTagMap_t; 39] =
    [{
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int)
                                 as uint8_t,
                         channel: 0x1 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int)
                                 as uint8_t,
                         channel: 0x2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 2 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int)
                                 as uint8_t,
                         channel: 0x3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 3 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int)
                                 as uint8_t,
                         channel: 0x4 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 4 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_2 as libc::c_int)
                                 as uint8_t,
                         channel: 0x1 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 5 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_2 as libc::c_int)
                                 as uint8_t,
                         channel: 0x2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 6 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_2 as libc::c_int)
                                 as uint8_t,
                         channel: 0x3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((0 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 7 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0x4 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 0 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0xc as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 1 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0x1 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 2 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_2 as libc::c_int)
                                 as uint8_t,
                         channel: 0xc as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 12 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0x3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 13 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0x5 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 14 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0x4 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((1 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 15 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0x5 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel: 0x6 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel: 0x7 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel: 0x8 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel: 0x9 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_2 as libc::c_int)
                                 as uint8_t,
                         channel: 0x5 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_2 as libc::c_int)
                                 as uint8_t,
                         channel: 0xb as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0xc as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0xd as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_4 as libc::c_int) as uint8_t,
                         channel: 0x7 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_4 as libc::c_int) as uint8_t,
                         channel: 0x8 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_4 as libc::c_int) as uint8_t,
                         channel: 0x9 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_4 as libc::c_int) as uint8_t,
                         channel: 0xa as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_4 as libc::c_int) as uint8_t,
                         channel: 0xb as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0xd as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_4 as libc::c_int) as uint8_t,
                         channel: 0x6 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0x2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0xe as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0xf as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0x10 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_3 as libc::c_int)
                                 as uint8_t,
                         channel: 0x3 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0x1 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_4 as libc::c_int)
                                 as uint8_t,
                         channel: 0x2 as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag: 0 as libc::c_int as ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int |
                                  (1 as libc::c_int) <<
                                      ADCDEV_2 as libc::c_int) as uint8_t,
                         channel: 0xa as libc::c_int as uint8_t,};
         init
     },
     {
         let mut init =
             adcTagMap_s{tag:
                             ((5 as libc::c_int + 1 as libc::c_int) <<
                                  4 as libc::c_int | 4 as libc::c_int) as
                                 ioTag_t,
                         devices:
                             ((1 as libc::c_int) << ADCDEV_1 as libc::c_int)
                                 as uint8_t,
                         channel: 0x5 as libc::c_int as uint8_t,};
         init
     }];
#[no_mangle]
pub unsafe extern "C" fn adcInit(mut config: *const adcConfig_t) {
    let mut ADC_InitStructure: ADC_InitTypeDef =
        ADC_InitTypeDef{ADC_ContinuousConvMode: 0,
                        ADC_Resolution: 0,
                        ADC_ExternalTrigConvEvent: 0,
                        ADC_ExternalTrigEventEdge: 0,
                        ADC_DataAlign: 0,
                        ADC_OverrunMode: 0,
                        ADC_AutoInjMode: 0,
                        ADC_NbrOfRegChannel: 0,};
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
    let mut adcChannelCount: uint8_t = 0 as libc::c_int as uint8_t;
    memset(&mut adcOperatingConfig as *mut [adcOperatingConfig_t; 4] as
               *mut libc::c_void, 0 as libc::c_int,
           ::core::mem::size_of::<[adcOperatingConfig_t; 4]>() as
               libc::c_ulong);
    if (*config).vbat.enabled {
        adcOperatingConfig[ADC_BATTERY as libc::c_int as usize].tag =
            (*config).vbat.ioTag
    }
    if (*config).rssi.enabled {
        adcOperatingConfig[ADC_RSSI as libc::c_int as usize].tag =
            (*config).rssi.ioTag
        //RSSI_ADC_CHANNEL;
    }
    if (*config).external1.enabled {
        adcOperatingConfig[ADC_EXTERNAL1 as libc::c_int as usize].tag =
            (*config).external1.ioTag
        //EXTERNAL1_ADC_CHANNEL;
    }
    if (*config).current.enabled {
        adcOperatingConfig[ADC_CURRENT as libc::c_int as usize].tag =
            (*config).current.ioTag
        //CURRENT_METER_ADC_CHANNEL;
    }
    let mut device: ADCDevice =
        adcDeviceByInstance((0x40000000 as libc::c_int as
                                 uint32_t).wrapping_add(0x10000000 as
                                                            libc::c_int as
                                                            libc::c_uint).wrapping_add(0
                                                                                           as
                                                                                           libc::c_int
                                                                                           as
                                                                                           libc::c_uint)
                                as *mut ADC_TypeDef);
    if device as libc::c_int == ADCINVALID as libc::c_int { return }
    let mut adc: adcDevice_t = adcHardware[device as usize];
    let mut adcActive: bool = 0 as libc::c_int != 0;
    let mut i: libc::c_int = 0 as libc::c_int;
    while i < ADC_CHANNEL_COUNT as libc::c_int {
        if adcVerifyPin(adcOperatingConfig[i as usize].tag, device) {
            adcActive = 1 as libc::c_int != 0;
            IOInit(IOGetByTag(adcOperatingConfig[i as usize].tag),
                   (OWNER_ADC_BATT as libc::c_int + i) as resourceOwner_e,
                   0 as libc::c_int as uint8_t);
            IOConfigGPIO(IOGetByTag(adcOperatingConfig[i as usize].tag),
                         (GPIO_Mode_AN as libc::c_int |
                              (0 as libc::c_int) << 2 as libc::c_int |
                              (GPIO_OType_OD as libc::c_int) <<
                                  4 as libc::c_int |
                              (GPIO_PuPd_NOPULL as libc::c_int) <<
                                  5 as libc::c_int) as ioConfig_t);
            adcOperatingConfig[i as usize].adcChannel =
                adcChannelByTag(adcOperatingConfig[i as usize].tag);
            let fresh0 = adcChannelCount;
            adcChannelCount = adcChannelCount.wrapping_add(1);
            adcOperatingConfig[i as usize].dmaIndex = fresh0;
            adcOperatingConfig[i as usize].sampleTime =
                0x7 as libc::c_int as uint8_t;
            adcOperatingConfig[i as usize].enabled = 1 as libc::c_int != 0
        }
        i += 1
    }
    if !adcActive { return }
    if device as libc::c_int == ADCDEV_1 as libc::c_int ||
           device as libc::c_int == ADCDEV_2 as libc::c_int {
        // enable clock for ADC1+2
        RCC_ADCCLKConfig(0x1b0 as libc::c_int as uint32_t);
        // 72 MHz divided by 256 = 281.25 kHz
    } else {
        // enable clock for ADC3+4
        RCC_ADCCLKConfig(0x10003600 as libc::c_int as uint32_t);
        // 72 MHz divided by 256 = 281.25 kHz
    }
    RCC_ClockCmd(adc.rccADC, ENABLE);
    dmaInit(dmaGetIdentifier(adc.DMAy_Channelx), OWNER_ADC,
            0 as libc::c_int as uint8_t);
    DMA_DeInit(adc.DMAy_Channelx);
    DMA_StructInit(&mut DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr =
        &mut (*adc.ADCx).DR as *mut uint32_t as uint32_t;
    DMA_InitStructure.DMA_MemoryBaseAddr = adcValues.as_mut_ptr() as uint32_t;
    DMA_InitStructure.DMA_DIR = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_BufferSize = adcChannelCount as uint16_t;
    DMA_InitStructure.DMA_PeripheralInc = 0 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryInc =
        if adcChannelCount as libc::c_int > 1 as libc::c_int {
            0x80 as libc::c_int as uint32_t
        } else { 0 as libc::c_int as uint32_t };
    DMA_InitStructure.DMA_PeripheralDataSize =
        0x100 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_MemoryDataSize = 0x400 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_Mode = 0x20 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_Priority = 0x2000 as libc::c_int as uint32_t;
    DMA_InitStructure.DMA_M2M = 0 as libc::c_int as uint32_t;
    DMA_Init(adc.DMAy_Channelx, &mut DMA_InitStructure);
    DMA_Cmd(adc.DMAy_Channelx, ENABLE);
    // calibrate
    ADC_VoltageRegulatorCmd(adc.ADCx, ENABLE);
    delay(10 as libc::c_int as timeMs_t);
    ADC_SelectCalibrationMode(adc.ADCx, 0 as libc::c_int as uint32_t);
    ADC_StartCalibration(adc.ADCx);
    while ADC_GetCalibrationStatus(adc.ADCx) as libc::c_uint !=
              RESET as libc::c_int as libc::c_uint {
    }
    ADC_VoltageRegulatorCmd(adc.ADCx, DISABLE);
    let mut ADC_CommonInitStructure: ADC_CommonInitTypeDef =
        ADC_CommonInitTypeDef{ADC_Mode: 0,
                              ADC_Clock: 0,
                              ADC_DMAAccessMode: 0,
                              ADC_DMAMode: 0,
                              ADC_TwoSamplingDelay: 0,};
    ADC_CommonStructInit(&mut ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = 0 as libc::c_int as uint32_t;
    ADC_CommonInitStructure.ADC_Clock = 0x30000 as libc::c_int as uint32_t;
    ADC_CommonInitStructure.ADC_DMAAccessMode =
        0x8000 as libc::c_int as uint32_t;
    ADC_CommonInitStructure.ADC_DMAMode = 0x2 as libc::c_int as uint32_t;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay =
        0 as libc::c_int as uint8_t;
    ADC_CommonInit(adc.ADCx, &mut ADC_CommonInitStructure);
    ADC_StructInit(&mut ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode =
        0x2000 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_Resolution = 0 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_ExternalTrigConvEvent =
        0 as libc::c_int as uint16_t as uint32_t;
    ADC_InitStructure.ADC_ExternalTrigEventEdge =
        0 as libc::c_int as uint16_t as uint32_t;
    ADC_InitStructure.ADC_DataAlign = 0 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_OverrunMode = 0 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_AutoInjMode = 0 as libc::c_int as uint32_t;
    ADC_InitStructure.ADC_NbrOfRegChannel = adcChannelCount;
    ADC_Init(adc.ADCx, &mut ADC_InitStructure);
    let mut rank: uint8_t = 1 as libc::c_int as uint8_t;
    let mut i_0: libc::c_int = 0 as libc::c_int;
    while i_0 < ADC_CHANNEL_COUNT as libc::c_int {
        if adcOperatingConfig[i_0 as usize].enabled {
            let fresh1 = rank;
            rank = rank.wrapping_add(1);
            ADC_RegularChannelConfig(adc.ADCx,
                                     adcOperatingConfig[i_0 as
                                                            usize].adcChannel,
                                     fresh1,
                                     adcOperatingConfig[i_0 as
                                                            usize].sampleTime);
        }
        i_0 += 1
    }
    ADC_Cmd(adc.ADCx, ENABLE);
    while ADC_GetFlagStatus(adc.ADCx,
                            0x1 as libc::c_int as uint16_t as uint32_t) as u64
              == 0 {
    }
    ADC_DMAConfig(adc.ADCx, 0x2 as libc::c_int as uint32_t);
    ADC_DMACmd(adc.ADCx, ENABLE);
    ADC_StartConversion(adc.ADCx);
}
unsafe extern "C" fn run_static_initializers() {
    adcHardware =
        [{
             let mut init =
                 adcDevice_s{ADCx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x10000000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut ADC_TypeDef,
                             rccADC:
                                 (((RCC_AHB as libc::c_int) <<
                                       5 as libc::c_int) as libc::c_long |
                                      (16 as libc::c_int *
                                           (0x10000000 as libc::c_int as
                                                uint32_t as libc::c_long >
                                                65535 as libc::c_long) as
                                               libc::c_int) as libc::c_long +
                                          ((8 as libc::c_int *
                                                (0x10000000 as libc::c_int as
                                                     uint32_t as libc::c_long
                                                     * 1 as libc::c_long >>
                                                     16 as libc::c_int *
                                                         (0x10000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long >
                                                              65535 as
                                                                  libc::c_long)
                                                             as libc::c_int >
                                                     255 as libc::c_int as
                                                         libc::c_long) as
                                                    libc::c_int) as
                                               libc::c_long +
                                               (8 as libc::c_int as
                                                    libc::c_long -
                                                    90 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x10000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x10000000
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
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             4 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long)
                                                    -
                                                    2 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x10000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x10000000
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
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             2 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long))))
                                     as rccPeriphTag_t,
                             DMAy_Channelx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x20000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x8
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut DMA_Channel_TypeDef,};
             init
         },
         {
             let mut init =
                 adcDevice_s{ADCx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x10000000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x100
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut ADC_TypeDef,
                             rccADC:
                                 (((RCC_AHB as libc::c_int) <<
                                       5 as libc::c_int) as libc::c_long |
                                      (16 as libc::c_int *
                                           (0x10000000 as libc::c_int as
                                                uint32_t as libc::c_long >
                                                65535 as libc::c_long) as
                                               libc::c_int) as libc::c_long +
                                          ((8 as libc::c_int *
                                                (0x10000000 as libc::c_int as
                                                     uint32_t as libc::c_long
                                                     * 1 as libc::c_long >>
                                                     16 as libc::c_int *
                                                         (0x10000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long >
                                                              65535 as
                                                                  libc::c_long)
                                                             as libc::c_int >
                                                     255 as libc::c_int as
                                                         libc::c_long) as
                                                    libc::c_int) as
                                               libc::c_long +
                                               (8 as libc::c_int as
                                                    libc::c_long -
                                                    90 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x10000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x10000000
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
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             4 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long)
                                                    -
                                                    2 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x10000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x10000000
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
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x10000000
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
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             2 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long))))
                                     as rccPeriphTag_t,
                             DMAy_Channelx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x20000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x408
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut DMA_Channel_TypeDef,};
             init
         },
         {
             let mut init =
                 adcDevice_s{ADCx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x10000000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x400
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut ADC_TypeDef,
                             rccADC:
                                 (((RCC_AHB as libc::c_int) <<
                                       5 as libc::c_int) as libc::c_long |
                                      (16 as libc::c_int *
                                           (0x20000000 as libc::c_int as
                                                uint32_t as libc::c_long >
                                                65535 as libc::c_long) as
                                               libc::c_int) as libc::c_long +
                                          ((8 as libc::c_int *
                                                (0x20000000 as libc::c_int as
                                                     uint32_t as libc::c_long
                                                     * 1 as libc::c_long >>
                                                     16 as libc::c_int *
                                                         (0x20000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long >
                                                              65535 as
                                                                  libc::c_long)
                                                             as libc::c_int >
                                                     255 as libc::c_int as
                                                         libc::c_long) as
                                                    libc::c_int) as
                                               libc::c_long +
                                               (8 as libc::c_int as
                                                    libc::c_long -
                                                    90 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x20000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x20000000
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
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x20000000
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
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x20000000
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
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             4 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             14 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             |
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long)
                                                    -
                                                    2 as libc::c_int as
                                                        libc::c_long /
                                                        ((0x20000000 as
                                                              libc::c_int as
                                                              uint32_t as
                                                              libc::c_long *
                                                              1 as
                                                                  libc::c_long
                                                              >>
                                                              16 as
                                                                  libc::c_int
                                                                  *
                                                                  (0x20000000
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
                                                              8 as libc::c_int
                                                                  *
                                                                  (0x20000000
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
                                                                       16 as
                                                                           libc::c_int
                                                                           *
                                                                           (0x20000000
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
                                                                       255 as
                                                                           libc::c_int
                                                                           as
                                                                           libc::c_long)
                                                                      as
                                                                      libc::c_int)
                                                             /
                                                             2 as libc::c_int
                                                                 as
                                                                 libc::c_long
                                                             +
                                                             1 as libc::c_int
                                                                 as
                                                                 libc::c_long))))
                                     as rccPeriphTag_t,
                             DMAy_Channelx:
                                 (0x40000000 as libc::c_int as
                                      uint32_t).wrapping_add(0x20000 as
                                                                 libc::c_int
                                                                 as
                                                                 libc::c_uint).wrapping_add(0x458
                                                                                                as
                                                                                                libc::c_int
                                                                                                as
                                                                                                libc::c_uint)
                                     as *mut DMA_Channel_TypeDef,};
             init
         }]
}
#[used]
#[cfg_attr(target_os = "linux", link_section = ".init_array")]
#[cfg_attr(target_os = "windows", link_section = ".CRT$XIB")]
#[cfg_attr(target_os = "macos", link_section = "__DATA,__mod_init_func")]
static INIT_ARRAY: [unsafe extern "C" fn(); 1] = [run_static_initializers];
